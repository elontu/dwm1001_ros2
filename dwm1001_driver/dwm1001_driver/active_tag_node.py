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

from collections import deque

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PointStamped, TransformStamped

import dwm1001
import serial
import time
import json
from std_msgs.msg import String

class ActiveTagNode(Node):
    def __init__(self) -> None:

        super().__init__("dwm_active", allow_undeclared_parameters=True)
        self.range_publisher = self.create_publisher(String, 'uwb_ranges_json', 10) #publisher for UWB ranges in JSON format
        self._declare_parameters()
        samples_param = self.get_parameter("samples").value
        if samples_param > 10:
            self.get_logger().warn("Number of samples exceeds maximum value. Capping at 10.")
            samples_param = 10
        self.get_logger().info(f"Performing moving average with {samples_param} samples.")
        self._position_buffer = deque(maxlen=samples_param)
        
        serial_port_param = self.get_parameter("serial_port").value
        self.get_logger().info(f"Provided serial port: '{serial_port_param}'")
        serial_handle = self._open_serial_port(serial_port_param)
        self.dwm_handle = dwm1001.ActiveTag(serial_handle)

        #---------------for debugging purposes only----------------
        self.get_logger().info("Waking up DWM1001...")
        is_ready = False
        # Try up to 500 times (total ~150 seconds) to cover boot time and beeping
        for i in range(150):
            # Send ENTER to wake the device and get the prompt
            self.dwm_handle.serial_handle.write(b'\r')
            time.sleep(1) #sleep for 1 second
            
            if self.dwm_handle.serial_handle.in_waiting > 0:
                # Read all waiting data (including the Copyright messages we saw in Putty)
                data = self.dwm_handle.serial_handle.read(self.dwm_handle.serial_handle.in_waiting)
                decoded_data = data.decode('utf-8', errors='ignore')
                
                # Check whether the device printed the prompt indicating readiness
                if "leaps>" in decoded_data:
                    self.get_logger().info(f"Device synchronized! (Prompt detected after {i+1} attempts)")
                    is_ready = True
                    break
            
            if (i+1) % 10 == 0:
                self.get_logger().info(f"Still waiting for prompt... (Attempt {i+1}/150)")

        if not is_ready:
            self.get_logger().error("FAILED to find 'leaps>' prompt. Device might be unresponsive.")

        # Final cleanup of the buffer from any leftover Copyright text before the real command
        #self.dwm_handle.serial_handle.reset_input_buffer()
        # cleaning input buffer
        #self.dwm_handle.serial_handle.reset_input_buffer()
        #self.dwm_handle.send_shell_command(dwm1001.ShellCommand.DOUBLE_ENTER)
        #----------------------------------------------------------
        self.dwm_handle.start_position_reporting()
        self.get_logger().info("Started position reporting.")
        tag_topic = f"~/output/{self.get_parameter('tag_id').value}"
        self.point_publisher = self.create_publisher(PointStamped, tag_topic, 1)
        self.get_logger().info(f"Publishing DWM postions on {tag_topic}")
        self.timer = self.create_timer(1 / 25, self.timer_callback)



    def _open_serial_port(self, serial_port: str) -> serial.Serial:
        if not serial_port:
            self._shutdown_fatal("No serial port specified.")
        try:
            serial_handle = serial.Serial(serial_port, baudrate=115_200)
            serial_handle.dtr = False  # Disable DTR to prevent resets
            serial_handle.rts = False  # Disable RTS to prevent resets
        except serial.SerialException:
            self._shutdown_fatal(f"Could not open serial port '{serial_port}'.")
        self.get_logger().info(f"Opened serial port: '{serial_port}'.")
        return serial_handle

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _declare_parameters(self):
        
        serial_port_descriptor = ParameterDescriptor(
            description="Device file or COM port associated with DWM1001",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        tag_id_descriptor = ParameterDescriptor(
            description="The ID for the particular DWM1001 device.",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )

        samples_descriptor = ParameterDescriptor(
            description="The number of samples used to perform the moving average. Max = 10",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        self.declare_parameter("samples", 3, samples_descriptor)
        self.declare_parameter("tag_id", "", tag_id_descriptor)
        self.declare_parameter("serial_port", "", serial_port_descriptor)

    def timer_callback(self):
        # Use 'serial_handle' instead of 'serial' as per the library definition
        if self.dwm_handle.serial_handle.in_waiting > 0:
            try:
                # Read a full line from the device - for debugging purposes
                line = self.dwm_handle.serial_handle.readline().decode('utf-8', errors='ignore').strip()
                parts = line.split()
                distances = {}

                for part in parts:
                    if '[' in part and '=' in part:
                        try:
                            anchor_id = part.split('[')[0]
                            distance_value = float(part.split('=')[-1])
                            distances[anchor_id] = distance_value

                        except Exception as e:
                            self.get_logger().warn(f"Parsing error: {e}")
                if distances:
                    self.get_logger().info(f"UWB Distances: {distances}")
                    #Convert distances dict to JSON and publish
                    msg = String()
                    msg.data = json.dumps(distances)
                    self.range_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error reading from DWM1001: {e}")
        else:
            pass

def main(args=None):
    rclpy.init(args=args)

    active_tag = ActiveTagNode()
    rclpy.spin(active_tag)

    active_tag.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
