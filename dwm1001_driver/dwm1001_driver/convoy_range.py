import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

#for now this code dont work as expected
class ConvoyRangeNode(Node):
    def __init__(self):
        super().__init__('convoy_range_finder')
        
        # Declare parameters so they exist in the ROS2 system
        self.declare_parameter('d1', 0.5)
        self.declare_parameter('d2', 0.5)
        
        # Fetch the values
        self.d1 = self.get_parameter('d1').get_parameter_value().double_value
        self.d2 = self.get_parameter('d2').get_parameter_value().double_value
        
        self.subscription = self.create_subscription(
            String,
            'uwb_ranges_json',
            self.range_callback,
            10)
        
        self.get_logger().info(f"Convoy Node Started. d1={self.d1}, d2={self.d2}")

    def range_callback(self, msg):
        try:
            # 1. Capture the raw data
            distances = json.loads(msg.data)
            
            # 2. Log EVERYTHING received for debugging
            # This helps you see if the IDs are '9966' or maybe 'DW9966'
            self.get_logger().info(f"Incoming JSON keys: {list(distances.keys())}")

            # 3. Try to extract your specific anchors
            r1 = distances.get('9966') 
            r2 = distances.get('996E') 

            if r1 is not None and r2 is not None:
                self.get_logger().info(f"MATCH! r1: {r1}m, r2: {r2}m")
                
                # Run the math
                rng, azimuth = self.convoy_range_azimuth(r1, r2, r1, r2, self.d1, self.d2)
                self.get_logger().info(f"CALC -> Range: {rng:.2f}m, Azimuth: {np.rad2deg(azimuth):.2f}°")
            else:
                # This tells you exactly what's missing
                self.get_logger().warn(f"Missing IDs. Need 9966/996E, got: {distances}")

        except Exception as e:
            self.get_logger().error(f"Callback failed: {e}")
                
     

    # --- Geometric Functions ---

    def median_theorem(self, a, b, d):
        """Calculates the length of the median to side d."""
        return np.sqrt((a**2 + b**2 - (d**2)/2)/2)

    def cosine_law_angle(self, a, b, c):
        """Calculates the angle opposite to side c using the Law of Cosines."""
        cos_val = (a**2 + b**2 - c**2) / (2 * a * b)
        # Clip value to avoid math domain errors due to UWB noise
        return np.arccos(np.clip(cos_val, -1.0, 1.0))

    def convoy_range_azimuth(self, r1, r2, r3, r4, d1, d2):
        """
        Calculates range and azimuth between vehicle centers.
        m1/m2 find distances to leader center.
        range finds distance between both vehicle centers.
        """
        # Distances from follower anchors to leader center
        m1 = self.median_theorem(r1, r2, d1)
        m2 = self.median_theorem(r3, r4, d1)
        
        # Final range between vehicle centers
        range_val = self.median_theorem(m1, m2, d2)

        # Azimuth calculation (Angle relative to vehicle heading)
        # Pi/2 offset assumes 0 is straight ahead
        azimuth = np.pi/2 - self.cosine_law_angle(range_val, d2/2, m2)

        return range_val, azimuth

def main(args=None):
    rclpy.init(args=args)
    node = ConvoyRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


