# Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import serial
import time
import subprocess
import re
import os
import stat
from . import dwm1001_forked as dwm1001_forked
from dwm1001_msg.msg import NamedValueArray, NamedValue


class ActiveTagNode(Node):
    def __init__(self) -> None:

        super().__init__("dwm_active", allow_undeclared_parameters=True)
        self.get_logger().debug("Initializing ActiveTagNode...")
        
        # Initialize device handles to None for cleanup safety
        self.serial_handle = None
        self.dwm_handle = None
        
        self._declare_parameters()
        self.get_logger().debug("Parameters declared")
        
        # Set logger level from parameter
        self._set_logger_level()
        
        # Get all parameters
        namespace = self.get_parameter("namespace").value
        serial_port_param = self.get_parameter("serial_port").value
        frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter("publish_rate").value
        wakeup_max_attempts = self.get_parameter("wakeup_max_attempts").value
        
        self.get_logger().debug(f"Parameter values - namespace: '{namespace}', serial_port: '{serial_port_param}', "
                                f"frame_id: '{frame_id}', publish_rate: {publish_rate} Hz, "
                                f"wakeup_max_attempts: {wakeup_max_attempts}")
        
        # Build topic names with namespace
        uwb_ranges_topic = f"{namespace}/{frame_id}/uwb_ranges"
        
        self.range_publisher = self.create_publisher(NamedValueArray, uwb_ranges_topic, 10)
        self.get_logger().debug(f"Created range_publisher on topic '{uwb_ranges_topic}'")
        
        self.get_logger().info(f"Provided serial port: '{serial_port_param}'")
        
        # Check and validate serial port availability before opening
        validated_port = self._check_serial_ports(serial_port_param)
        
        # Store serial handle as instance variable for cleanup
        self.serial_handle = self._open_serial_port(validated_port)
        self.get_logger().debug(f"Serial handle created: {self.serial_handle}")
        
        # Get max initialization attempts parameter
        max_init_attempts = int(self.get_parameter("init_max_attempts").value)
        self.get_logger().debug(f"Max initialization attempts: {max_init_attempts}")
        
        # Retry initialization up to max_init_attempts times
        self.dwm_handle = None
        for attempt in range(1, max_init_attempts + 1):
            try:
                self.get_logger().info(f"Initializing DWM1001 device (attempt {attempt}/{max_init_attempts})...")
                self.dwm_handle = dwm1001_forked.ActiveTag(self.serial_handle)
                self.get_logger().info(f"DWM1001 ActiveTag handle created successfully on attempt {attempt}")
                break
            except (OSError, IOError, serial.SerialException) as e:
                if attempt < max_init_attempts:
                    self.get_logger().warn(f"I/O error on attempt {attempt}/{max_init_attempts}: {e}. Retrying in 0.5 seconds...")
                    time.sleep(0.5)
                else:
                    # Cleanup before raising error
                    self._cleanup_device()
                    error_msg = f"I/O error when initializing DWM1001 device after {max_init_attempts} attempts: {e}. " \
                               f"Check if device is connected, powered on, and not in use by another process. " \
                               f"Try unplugging and replugging the USB device."
                    self.get_logger().error(error_msg)
                    raise RuntimeError(error_msg) from e
            except Exception as e:
                if attempt < max_init_attempts:
                    self.get_logger().warn(f"Unexpected error on attempt {attempt}/{max_init_attempts}: {e}. Retrying in 0.5 seconds...")
                    time.sleep(0.5)
                else:
                    # Cleanup before raising error
                    self._cleanup_device()
                    error_msg = f"Unexpected error when initializing DWM1001 device after {max_init_attempts} attempts: {e}"
                    self.get_logger().error(error_msg)
                    raise RuntimeError(error_msg) from e
        
        if self.dwm_handle is None:
            # Cleanup before raising error
            self._cleanup_device()
            error_msg = f"Failed to initialize DWM1001 device after {max_init_attempts} attempts"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Flag to ensure wakeup runs only once
        self._wakeup_done = False
        
        # Get publish rate parameter and create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Timer created with publish rate: {publish_rate} Hz (period: {timer_period}s)")
        self.get_logger().debug(f"Timer callback will be called every {timer_period} seconds")

    def _check_serial_ports(self, configured_port: str) -> str:
        """
        Check available ttyACM devices and validate/update serial_port configuration.
        Returns the validated port path to use.
        """
        self.get_logger().debug(f"Checking serial ports. Configured port: '{configured_port}'")
        
        # Run command to list ttyACM devices and dmesg output
        cmd = "ls -la /dev/ttyACM* 2>/dev/null; echo '---'; dmesg | grep -i 'ttyACM\\|cdc_acm'"
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode != 0:
                error_msg = f"Failed to check serial ports: {result.stderr}"
                self.get_logger().error(error_msg)
                raise RuntimeError(error_msg)
            
            output = result.stdout
            self.get_logger().debug(f"Serial port check output:\n{output}")
            
            # Extract ttyACM device paths from output
            ttyacm_pattern = r'/dev/ttyACM\d+'
            found_ports = re.findall(ttyacm_pattern, output)
            
            if not found_ports:
                error_msg = "No ttyACM devices found. Check device connection."
                self.get_logger().error(error_msg)
                raise RuntimeError(error_msg)
            
            # Remove duplicates and sort
            found_ports = sorted(list(set(found_ports)))
            
            # Check if configured serial port is in the found ports
            port_to_use = configured_port if configured_port in found_ports else found_ports[0]
            if configured_port not in found_ports:
                self.get_logger().warn(
                    f"Configured serial port {configured_port} not found. "
                    f"Using first available port: {port_to_use}"
                )
            
            # Validate port is writable and not locked
            self._validate_port_writable(port_to_use)
            
            self.get_logger().info(f"Found serial port {port_to_use} as listed under configuration")
            return port_to_use
                
        except subprocess.TimeoutExpired:
            error_msg = "Timeout while checking serial ports"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        except Exception as e:
            error_msg = f"Error checking serial ports: {str(e)}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
    
    def _validate_port_writable(self, port_path: str):
        """
        Validate that the serial port is writable and not locked by another process.
        """
        # Check if file exists and is a character device
        if not os.path.exists(port_path):
            error_msg = f"Serial port {port_path} does not exist"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        if not stat.S_ISCHR(os.stat(port_path).st_mode):
            error_msg = f"{port_path} is not a character device"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Check if port is locked by another process
        try:
            lsof_result = subprocess.run(
                ['lsof', port_path],
                capture_output=True,
                text=True,
                timeout=2
            )
            if lsof_result.returncode == 0 and lsof_result.stdout.strip():
                processes = lsof_result.stdout.strip().split('\n')[1:]  # Skip header
                pid_list = [line.split()[1] for line in processes if line.strip()]
                error_msg = f"Serial port {port_path} is locked by process(es): {', '.join(set(pid_list))}"
                self.get_logger().error(error_msg)
                raise RuntimeError(error_msg)
        except FileNotFoundError:
            # lsof not available, try fuser instead
            try:
                fuser_result = subprocess.run(
                    ['fuser', port_path],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if fuser_result.returncode == 0:
                    error_msg = f"Serial port {port_path} is in use by another process"
                    self.get_logger().error(error_msg)
                    raise RuntimeError(error_msg)
            except FileNotFoundError:
                self.get_logger().debug("lsof/fuser not available, skipping lock check")
        except subprocess.TimeoutExpired:
            self.get_logger().debug("Timeout checking for locked port (may be OK)")
        
        # Check write permissions
        if not os.access(port_path, os.W_OK):
            error_msg = f"Serial port {port_path} is not writable. Check permissions (user may need dialout group)"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        self.get_logger().debug(f"Serial port {port_path} is writable and available")

    def _open_serial_port(self, serial_port: str) -> serial.Serial:
        self.get_logger().debug(f"Attempting to open serial port: '{serial_port}'")
        if not serial_port:
            self._shutdown_fatal("No serial port specified.")
        try:
            self.get_logger().debug(f"Creating Serial connection with baudrate=115200")
            serial_handle = serial.Serial(serial_port, baudrate=115_200)
            serial_handle.dtr = False  # Disable DTR to prevent resets
            serial_handle.rts = False  # Disable RTS to prevent resets
            self.get_logger().debug(f"Serial port opened successfully. DTR={serial_handle.dtr}, RTS={serial_handle.rts}")
        except serial.SerialException as e:
            self.get_logger().error(f"SerialException when opening port '{serial_port}': {e}")
            self._shutdown_fatal(f"Could not open serial port '{serial_port}'.")
        self.get_logger().info(f"Opened serial port: '{serial_port}'.")
        return serial_handle

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _wakeup(self) -> None:
        """Wake up DWM1001 device and wait for readiness prompt."""
        self.get_logger().info("Waking up DWM1001...")
        is_ready = False
        
        # Get wakeup max attempts parameter
        max_attempts = int(self.get_parameter("wakeup_max_attempts").value)
        self.get_logger().debug(f"Wakeup max attempts: {max_attempts}")
        
        # Try up to max_attempts times (total ~max_attempts seconds) to cover boot time and beeping
        decoded_data = ""
        i = 0
        while "leaps>" not in decoded_data : #and i < max_attempts TODO delete max_attempts
            i += 1
            # Send ENTER to wake the device and get the prompt
            self.get_logger().debug(f"Wakeup attempt {i}/{max_attempts}: Sending ENTER command")
            self.dwm_handle.serial_handle.write(b'\r')
            time.sleep(0.5)  # sleep for 0.5 second
            
            bytes_waiting = self.dwm_handle.serial_handle.in_waiting
            self.get_logger().debug(f"Bytes waiting in serial buffer: {bytes_waiting}")
            
            decoded_data = ""
            if bytes_waiting > 0:
                # Read all waiting data (including the Copyright messages we saw in Putty)
                data = self.dwm_handle.serial_handle.read(bytes_waiting)
                decoded_data = data.decode('utf-8', errors='ignore')
                self.get_logger().debug(f"Received data {decoded_data} (length={len(decoded_data)}): {repr(decoded_data[:100])}")  # Log first 100 chars
            
            if i % 10 == 0:
                self.get_logger().info(f"Still waiting for prompt... (Attempt {i}/{max_attempts})")
            else:
                self.get_logger().debug(f"Attempt {i}/{max_attempts}: No prompt detected yet")

        if "leaps>" in decoded_data:
            self.get_logger().info(f"Device synchronized! (Prompt detected after {i} attempts)")
            self.get_logger().debug(f"Full response: {repr(decoded_data)}")
            is_ready = True
        else:
            is_ready = False

        if not is_ready:
            self.get_logger().error(f"FAILED to find 'leaps>' prompt after {max_attempts} attempts. Device might be unresponsive.")
            self.get_logger().debug("Wakeup sequence completed unsuccessfully")
        else:
            self.get_logger().debug("Wakeup sequence completed successfully")
            self.dwm_handle.start_position_reporting()
            self.get_logger().info("Started position reporting.")
            self.get_logger().debug("Position reporting started via dwm1001 library")

    def _read_serial_data(self) -> None:
        """Read and process serial data from DWM1001 device."""
        # Use 'serial_handle' instead of 'serial' as per the library definition
        bytes_waiting = self.dwm_handle.serial_handle.in_waiting
        self.get_logger().debug(f"Serial buffer bytes waiting: {bytes_waiting}")
        
        if bytes_waiting > 0:
            try:
                # Read a full line from the device - for debugging purposes
                self.get_logger().debug("Reading line from serial port")
                line = self.dwm_handle.serial_handle.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().debug(f"Raw line received (length={len(line)}): {repr(line)}")
                
                parts = line.split()
                self.get_logger().debug(f"Line split into {len(parts)} parts: {parts}")
                distances = {}

                for part in parts:
                    if '[' in part and '=' in part:
                        try:
                            anchor_id = part.split('[')[0]
                            distance_value = float(part.split('=')[-1])
                            distances[anchor_id] = distance_value
                            self.get_logger().debug(f"Parsed anchor: {anchor_id} = {distance_value} m")

                        except Exception as e:
                            self.get_logger().warn(f"Parsing error for part '{part}': {e}")
                            self.get_logger().debug(f"Exception details: {type(e).__name__}: {str(e)}")
                
                if distances:
                    self.get_logger().info(f"UWB Distances: {distances}")
                    
                    # Create NamedValueArray message
                    msg = NamedValueArray()
                    
                    # Set header with timestamp and frame_id
                    msg.header.stamp = self.get_clock().now().to_msg()
                    frame_id = self.get_parameter('frame_id').value
                    msg.header.frame_id = frame_id
                    
                    # Convert distances dict to NamedValue array
                    msg.data = []
                    for anchor_id, distance_value in distances.items():
                        named_value = NamedValue()
                        named_value.name = anchor_id
                        named_value.value = float(distance_value)
                        msg.data.append(named_value)
                        self.get_logger().debug(f"Added NamedValue: name='{anchor_id}', value={distance_value}")
                    
                    self.get_logger().debug(f"Publishing NamedValueArray with {len(msg.data)} entries")
                    self.range_publisher.publish(msg)
                    self.get_logger().debug(f"Published message to 'uwb_ranges' topic")
                else:
                    self.get_logger().debug("No valid distances found in line, skipping publish")
            except Exception as e:
                self.get_logger().error(f"Error reading from DWM1001: {e}")
                self.get_logger().debug(f"Exception type: {type(e).__name__}, details: {str(e)}")
        else:
            self.get_logger().debug("No data waiting in serial buffer")

    def _declare_parameters(self):
        
        namespace_descriptor = ParameterDescriptor(
            description="ROS namespace for topics (default: empty string, no namespace)",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        serial_port_descriptor = ParameterDescriptor(
            description="Device file or COM port associated with DWM1001 (default: /dev/ttyACM*)",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        frame_id_descriptor = ParameterDescriptor(
            description="The frame ID for the particular DWM1001 device (default: dwm1001)",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        publish_rate_descriptor = ParameterDescriptor(
            description="Publish rate in Hz for timer callback (default: 25.0 Hz)",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        
        wakeup_max_attempts_descriptor = ParameterDescriptor(
            description="Maximum number of attempts to wake up DWM1001 device (default: 150)",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=True,
        )
        
        debug_level_descriptor = ParameterDescriptor(
            description="Logging level: DEBUG, INFO, WARN, ERROR, FATAL (default: INFO)",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        init_max_attempts_descriptor = ParameterDescriptor(
            description="Maximum number of attempts to initialize DWM1001 device (default: 5)",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=True,
        )
        
        self.declare_parameter("namespace", "dwm1001", namespace_descriptor)
        self.declare_parameter("frame_id", "dwm1001", frame_id_descriptor)
        self.declare_parameter("serial_port", "/dev/ttyACM1", serial_port_descriptor)
        self.declare_parameter("publish_rate", 25.0, publish_rate_descriptor)
        self.declare_parameter("wakeup_max_attempts", 150, wakeup_max_attempts_descriptor)
        self.declare_parameter("debug_level", "INFO", debug_level_descriptor)
        self.declare_parameter("init_max_attempts", 5, init_max_attempts_descriptor)

    def _set_logger_level(self):
        """Set the logger level based on the debug_level parameter."""
        debug_level_str = self.get_parameter("debug_level").value.upper()
        
        level_mapping = {
            "DEBUG": LoggingSeverity.DEBUG,
            "INFO": LoggingSeverity.INFO,
            "WARN": LoggingSeverity.WARN,
            "WARNING": LoggingSeverity.WARN,
            "ERROR": LoggingSeverity.ERROR,
            "FATAL": LoggingSeverity.FATAL,
        }
        
        if debug_level_str in level_mapping:
            self.get_logger().set_level(level_mapping[debug_level_str])
            self.get_logger().info(f"Logger level set to: {debug_level_str}")
        else:
            self.get_logger().warn(f"Invalid debug_level '{debug_level_str}'. Valid values: DEBUG, INFO, WARN, ERROR, FATAL. Using INFO.")
            self.get_logger().set_level(LoggingSeverity.INFO)

    def _cleanup_device(self):
        """Clean up device resources - stop position reporting and close serial port."""
        try:
            if hasattr(self, 'dwm_handle') and self.dwm_handle is not None:
                try:
                    self.get_logger().debug("Stopping position reporting...")
                    self.dwm_handle.stop_position_reporting()
                    self.get_logger().debug("Position reporting stopped")
                except Exception as e:
                    self.get_logger().warn(f"Error stopping position reporting: {e}")
                
                try:
                    self.get_logger().debug("Exiting shell mode...")
                    self.dwm_handle.exit_shell_mode()
                    self.get_logger().debug("Shell mode exited")
                except Exception as e:
                    self.get_logger().warn(f"Error exiting shell mode: {e}")
        except Exception as e:
            self.get_logger().warn(f"Error during dwm_handle cleanup: {e}")
        
        try:
            if hasattr(self, 'serial_handle') and self.serial_handle is not None:
                if self.serial_handle.is_open:
                    self.get_logger().info(f"Closing serial port: {self.serial_handle.port}")
                    self.serial_handle.close()
                    self.get_logger().debug("Serial port closed")
        except Exception as e:
            self.get_logger().warn(f"Error closing serial port: {e}")

    def destroy_node(self):
        """Override destroy_node to ensure proper cleanup."""
        self.get_logger().debug("Destroying ActiveTagNode, cleaning up device...")
        self._cleanup_device()
        super().destroy_node()
        self.get_logger().debug("ActiveTagNode destroyed")

    def timer_callback(self):
        self.get_logger().debug("Timer callback triggered")
        
        # Run wakeup once using boolean flag
        if not self._wakeup_done:
            self.get_logger().debug("Wakeup not done yet, calling _wakeup()")
            self._wakeup()
            self._wakeup_done = True
            self.get_logger().debug("Wakeup completed, flag set to True")
        
        # Read and process serial data
        self._read_serial_data()

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger("dwm_active").debug("Starting DWM1001 ActiveTagNode main()")

    try:
        active_tag = ActiveTagNode()
        rclpy.logging.get_logger("dwm_active").debug("ActiveTagNode created, starting spin")
    except RuntimeError as e:
        rclpy.logging.get_logger("dwm_active").fatal(f"Failed to initialize node: {e}")
        rclpy.shutdown()
        exit(1)
    except Exception as e:
        rclpy.logging.get_logger("dwm_active").fatal(f"Unexpected error during node initialization: {e}")
        rclpy.shutdown()
        exit(1)
    
    try:
        rclpy.spin(active_tag)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("dwm_active").info("Keyboard interrupt received, shutting down")
    except Exception as e:
        rclpy.logging.get_logger("dwm_active").error(f"Exception during spin: {e}")
        rclpy.logging.get_logger("dwm_active").debug(f"Exception details: {type(e).__name__}: {str(e)}")
    finally:
        rclpy.logging.get_logger("dwm_active").debug("Destroying node and shutting down")
        active_tag.destroy_node()
        rclpy.shutdown()
        rclpy.logging.get_logger("dwm_active").debug("Shutdown complete")


if __name__ == "__main__":
    main()
