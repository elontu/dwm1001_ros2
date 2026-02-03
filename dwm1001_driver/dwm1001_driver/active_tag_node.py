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

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import dwm1001
import serial
import time
from dwm1001_msg.msg import NamedValueArray, NamedValue


class ActiveTagNode(Node):
    def __init__(self) -> None:

        super().__init__("dwm_active", allow_undeclared_parameters=True)
        self.get_logger().debug("Initializing ActiveTagNode...")
        
        self._declare_parameters()
        self.get_logger().debug("Parameters declared")
        
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
        
        serial_handle = self._open_serial_port(serial_port_param)
        self.get_logger().debug(f"Serial handle created: {serial_handle}")
        
        self.dwm_handle = dwm1001.ActiveTag(serial_handle)
        self.get_logger().debug("DWM1001 ActiveTag handle created")
        
        # Flag to ensure wakeup runs only once
        self._wakeup_done = False
        
        # Get publish rate parameter and create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Timer created with publish rate: {publish_rate} Hz (period: {timer_period}s)")
        self.get_logger().debug(f"Timer callback will be called every {timer_period} seconds")

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
                self.get_logger().debug(f"Received data (length={len(decoded_data)}): {repr(decoded_data[:100])}")  # Log first 100 chars
            
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
            description="Device file or COM port associated with DWM1001 (default: /dev/ttyACM0)",
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
        
        self.declare_parameter("namespace", "dwm1001", namespace_descriptor)
        self.declare_parameter("frame_id", "dwm1001", frame_id_descriptor)
        self.declare_parameter("serial_port", "/dev/ttyACM0", serial_port_descriptor)
        self.declare_parameter("publish_rate", 25.0, publish_rate_descriptor)
        self.declare_parameter("wakeup_max_attempts", 150, wakeup_max_attempts_descriptor)

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

    active_tag = ActiveTagNode()
    rclpy.logging.get_logger("dwm_active").debug("ActiveTagNode created, starting spin")
    
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