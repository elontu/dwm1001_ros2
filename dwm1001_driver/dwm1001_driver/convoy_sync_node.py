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

"""
ROS2 node that subscribes to two UWB range topics (left and right), synchronizes
them with ApproximateTimeSynchronizer, applies convoy_range-style calculation.
# and publishes ConvoyRangeStamped (x_m, y_m, r, a, r_geo, a_geo).
"""

import math
import struct
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import message_filters
from sensor_msgs.msg import PointCloud2, PointField

from builtin_interfaces.msg import Time
from dwm1001_msg.msg import NamedValueArray
# from dwm1001_msg.msg import ConvoyRangeStamped
        
# --- Insert at top of file ---
class StaticTransformFetcher:
    """
    Utility/static class for looking up and caching static transforms between frames.
    """
    # Simple in-memory cache to persist transforms for the life of the node/process.
    _tf_cache = {}

    @staticmethod
    def get_transform(node, target_frame: str, source_frame: str, is_spin_once: bool = False):
        """
        Get the transform from target_frame to source_frame.
        Returns (transform_matrix_4x4, trans) or (None, None) on failure.
        transform_matrix is a numpy array; trans is the geometry_msgs TransformStamped.
        The result is cached for later queries (per frame-pair).
        """
        key = (target_frame, source_frame)
        if key in StaticTransformFetcher._tf_cache:
            return StaticTransformFetcher._tf_cache[key]

        import rclpy
        from rclpy.duration import Duration
        import time
        try:
            import tf2_ros
            from tf_transformations import quaternion_matrix
        except ImportError:
            node.get_logger().error("Required tf2_ros or tf_transformations not found!")
            return None, None

        # Set up tf2 components if not already present on node
        if not hasattr(node, 'tf_buffer'):
            node.tf_buffer = tf2_ros.Buffer()
            node.tf_listener = tf2_ros.TransformListener(node.tf_buffer, node, spin_thread=True)

        max_attempts = 10
        wait_time_per_attempt = 0.5
        for attempt in range(max_attempts):
            try:
                trans = node.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    Duration(seconds=0.1)
                )
                t = trans.transform.translation
                r = trans.transform.rotation
                transform_matrix = quaternion_matrix([r.x, r.y, r.z, r.w])
                transform_matrix[:3, 3] = [t.x, t.y, t.z]
                node.get_logger().info(
                    f'(Static) {target_frame} <- {source_frame} transform computed:\n{transform_matrix}'
                )
                result = (transform_matrix, trans)
                StaticTransformFetcher._tf_cache[key] = result
                return result
            except Exception as ex:
                node.get_logger().warn(
                    f'Attempt {attempt+1}/{max_attempts}: Static {target_frame} <- {source_frame} unavailable: {ex}'
                )
                if is_spin_once:
                    rclpy.spin_once(node, timeout_sec=wait_time_per_attempt)
                else:
                    time.sleep(wait_time_per_attempt)
        node.get_logger().error(
            f'Failed to retrieve static transform {target_frame} <- {source_frame} after {max_attempts} attempts.'
        )
        StaticTransformFetcher._tf_cache[key] = (None, None)
        return None, None
# --- End insert at top of file ---
        



_HALF_PI = math.pi / 2.0


def median_theorem(a: float, b: float, d: float) -> float:
    """Length of median to side d in triangle with sides a, b, d (law of cosines)."""
    a, b, d = float(a), float(b), float(d)
    half_d2 = (d * d) * 0.5
    return math.sqrt(((a * a + b * b) - half_d2) * 0.5)


def cosine_law_angle(a: float, b: float, c: float) -> float:
    """Angle opposite side c in triangle with sides a, b, c [radians]. Returns 0 if degenerate."""
    a, b, c = float(a), float(b), float(c)
    denom = 2.0 * a * b
    if denom == 0.0:
        return 0.0
    val = (a * a + b * b - c * c) / denom
    val = max(-1.0, min(1.0, val))
    return math.acos(val)


def cosine_law_line(a: float, b: float, alpha: float) -> float:
    """Length of side opposite angle alpha in triangle (law of cosines)."""
    a, b, alpha = float(a), float(b), float(alpha)
    return math.sqrt(a * a + b * b - 2.0 * a * b * math.cos(alpha))


def sine_law(a: float, b: float, beta: float) -> float:
    """Angle alpha such that a/sin(alpha) = b/sin(beta) [radians]. Returns 0 if b is zero."""
    a, b, beta = float(a), float(b), float(beta)
    if b == 0.0:
        return 0.0
    ratio = (a * math.sin(beta)) / b
    ratio = max(-1.0, min(1.0, ratio))
    return math.asin(ratio)


def get_coord(r1: float, r2: float, d: float) -> Tuple[float, float]:
    """
    (x, y) of point given distances r1, r2 to two anchors at (0,0) and (d,0).
    Raises ValueError if d is zero or geometry is invalid (e.g. negative radicand).
    """
    r1, r2, d = float(r1), float(r2), float(d)
    if d == 0.0:
        raise ValueError("d must not be 0.")
    two_d = 2.0 * d
    x = abs((r1 * r1 - r2 * r2 + d * d) / two_d)
    radicand = r1 * r1 - x * x
    if radicand < 0.0:
        radicand = 0.0
    y = math.sqrt(radicand)
    return x, y


def get_range_angle(x: float, y: float, d: float, l: float) -> Tuple[float, float]:
    """Range r and angle a [degrees] from convoy geometry (midpoint x,y, anchor spacing d, lidar offset l)."""
    x, y, d, l = float(x), float(y), float(d), float(l)
    half_d = d * 0.5
    dx = abs(x - half_d)
    init_r = math.sqrt((x - half_d) ** 2 + y * y)
    init_a = math.atan2(dx, y)
    r = cosine_law_line(init_r, l, init_a)
    a = math.degrees(math.atan2(dx, y - l))
    return r, a


def _mean_timestamp(t1: Time, t2: Time) -> Time:
    """Return the mean of two timestamps (sec + nanosec)."""
    ns1 = t1.sec * 10**9 + t1.nanosec
    ns2 = t2.sec * 10**9 + t2.nanosec
    mean_ns = (ns1 + ns2) // 2
    out = Time()
    out.sec = int(mean_ns // 10**9)
    out.nanosec = int(mean_ns % 10**9)
    return out


def _extract_r1_r2(msg: NamedValueArray) -> Tuple[Optional[float], Optional[float]]:
    """Extract first two range values from a NamedValueArray (left: r1, r2). Returns (None, None) if < 2 entries."""
    if len(msg.data) < 2:
        return None, None
    return float(msg.data[0].value), float(msg.data[1].value)


def convoy_range_from_ranges(
    r1: float, r2: float, r3: float, r4: float, d: float, l: float
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """
    Compute convoy range outputs from four ranges (left r1,r2; right r3,r4) and geometry d, l.
    Returns (x_m, y_m, r, a, r_geo, a_geo) or None if invalid.
    """
    if d <= 0.0 or l < 0.0:
        return None
    if not all(r >= 0.0 for r in (r1, r2, r3, r4)):
        return None
    try:
        # Analytic: midpoint and range/angle (left tag at 0,0)
        x1, y1 = get_coord(r1, r2, d)
        x2, y2 = get_coord(r3, r4, d)
        x_m = (x1 + x2) * 0.5
        y_m = (y1 + y2) * 0.5
        r, a = get_range_angle(x_m, y_m, d, l)

        # Geometric
        m1 = median_theorem(r1, r2, d)
        m2 = median_theorem(r3, r4, d)
        m = median_theorem(m1, m2, d)
        half_d = d * 0.5
        alpha = _HALF_PI - cosine_law_angle(m, half_d, m2)
        r_geo = cosine_law_line(l, m, alpha)
        a_geo = math.degrees(sine_law(m, r_geo, alpha) - _HALF_PI)

        result = (x_m, y_m, r, a, r_geo, a_geo)
        if not all(math.isfinite(v) for v in result):
            return None
        return result
    except (ValueError, ZeroDivisionError, FloatingPointError):
        return None


class ConvoySyncNode(Node):
    """Subscribes to two uwb_ranges topics, syncs them, runs convoy_range."""

    def __init__(self) -> None:
        super().__init__("convoy_sync", allow_undeclared_parameters=True)
        self._declare_parameters()

        left_topic = self.get_parameter("left_uwb_ranges_topic").value
        right_topic = self.get_parameter("right_uwb_ranges_topic").value
        self.d = self.get_parameter("d").value
        self.l = self.get_parameter("l").value
        frame_id = self.get_parameter("frame_id").value
        sync_slop = self.get_parameter("sync_slop").value
        queue_size = self.get_parameter("sync_queue_size").value

        # self.convoy_publisher = self.create_publisher(
        #     ConvoyRangeStamped,
        #     f"/convoy_sync/{frame_id}/convoy_range",
        #     10,
        # )

        self.left_sub = message_filters.Subscriber(self, NamedValueArray, left_topic)
        self.right_sub = message_filters.Subscriber(self, NamedValueArray, right_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self.sync.registerCallback(self._sync_callback)

        # Transform base_link <- left_tag.frame_id; computed once on first sync callback
        self._base_link_frame = self.get_parameter("base_link_frame").value
        self._tf_base_link_from_left_tag: Optional[np.ndarray] = None
        self._trans_base_link_from_left_tag = None  # geometry_msgs TransformStamped

        # Optional debug: publish convoy midpoint as a single-point PointCloud2
        # Use BEST_EFFORT so publish() never blocks (RELIABLE can block when subscriber is slow)
        debug_pc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._debug_pc_pub = self.create_publisher(
            PointCloud2,
            "convoy_sync/debug_pointcloud",
            debug_pc_qos,
        )

        self.get_logger().info(
            f"ConvoySync: subscribed to left={left_topic}, right={right_topic} (d={self.d}, l={self.l})"
        )

    def _declare_parameters(self):
        self.declare_parameter(
            "left_uwb_ranges_topic",
            "/dwm10011_left/tag_left/uwb_ranges",
            ParameterDescriptor(description="Left UWB ranges topic (NamedValueArray)", type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "right_uwb_ranges_topic",
            "/dwm10011_right/tag_right/uwb_ranges",
            ParameterDescriptor(description="Right UWB ranges topic (NamedValueArray)", type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "d",
            2.12,
            ParameterDescriptor(description="Distance between two anchors [m]", type=ParameterType.PARAMETER_DOUBLE),
        )
        self.declare_parameter(
            "l",
            2.0,
            ParameterDescriptor(description="Distance between Lidar and UWB [m]", type=ParameterType.PARAMETER_DOUBLE),
        )
        self.declare_parameter(
            "frame_id",
            "convoy",
            ParameterDescriptor(description="Frame ID for published messages", type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "sync_slop",
            0.05,
            ParameterDescriptor(description="ApproximateTime sync slop [s]", type=ParameterType.PARAMETER_DOUBLE),
        )
        self.declare_parameter(
            "sync_queue_size",
            10,
            ParameterDescriptor(description="Sync queue size", type=ParameterType.PARAMETER_INTEGER),
        )
        self.declare_parameter(
            "base_link_frame",
            "base_link",
            ParameterDescriptor(description="Target frame for transform (e.g. base_link)", type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "publish_debug_pointcloud",
            True,
            ParameterDescriptor(description="If true, publish (x_m, y_m, z_m) as a PointCloud2 for debugging", type=ParameterType.PARAMETER_BOOL),
        )

    def _sync_callback(self, left_msg: NamedValueArray, right_msg: NamedValueArray):
        # Fetch transform base_link <- left_tag.frame_id once
        if self._tf_base_link_from_left_tag is None:
            left_tag_frame = left_msg.header.frame_id
            self._tf_base_link_from_left_tag, self._trans_base_link_from_left_tag = (
                StaticTransformFetcher.get_transform(self, self._base_link_frame, left_tag_frame)
            )
            if self._tf_base_link_from_left_tag is None:
                self.get_logger().warn(
                    f"Could not get transform {self._base_link_frame} <- {left_tag_frame}; skipping."
                )
                return

        self.get_logger().info(f"Sync callback: left_msg: {left_msg}")
        self.get_logger().info(f"Sync callback: right_msg: {right_msg}")
        range_left1, range_left2 = _extract_r1_r2(left_msg)
        range_right1, range_right2 = _extract_r1_r2(right_msg)
        if range_left1 is None or range_right1 is None:
            self.get_logger().warn("Sync callback: need at least 2 entries per topic; skipping.")
            return

        result = convoy_range_from_ranges(range_left1, range_left2, range_right1, range_right2, self.d, self.l)
        if result is None:
            self.get_logger().warn("Convoy range computation failed (invalid geometry); skipping.")
            return
        x_m, y_m, r, a, r_geo, a_geo = result
        z_m = self._trans_base_link_from_left_tag.transform.translation.z
        # # Apply cached transform: point (x_m, y_m, 0) in left_tag frame -> base_link
        # p_left = np.array([x_m, y_m, 0.0, 1.0])
        # p_base = self._tf_base_link_from_left_tag @ p_left
        # x_m_base, y_m_base, z_m_base = float(p_base[0]), float(p_base[1]), float(p_base[2])

        return 
        
        if self.get_parameter("publish_debug_pointcloud").value:
            pc = PointCloud2()
            pc.header.stamp = _mean_timestamp(left_msg.header.stamp, right_msg.header.stamp)
            pc.header.frame_id = self._base_link_frame
            pc.height = 1
            pc.width = 1
            pc.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            pc.is_bigendian = False
            pc.point_step = 12
            pc.row_step = 12
            pc.data = struct.pack("fff", x_m, y_m, z_m)
            self._debug_pc_pub.publish(pc)

        # out = ConvoyRangeStamped()
        # out.header.stamp = _mean_timestamp(left_msg.header.stamp, right_msg.header.stamp)
        # out.header.frame_id = self._base_link_frame
        # out.x_m = x_m_base
        # out.y_m = y_m_base
        # out.r = r
        # out.a = a
        # out.r_geo = r_geo
        # out.a_geo = a_geo
        # self.convoy_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConvoySyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
