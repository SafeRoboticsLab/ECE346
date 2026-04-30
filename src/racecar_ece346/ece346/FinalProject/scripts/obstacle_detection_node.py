#!/usr/bin/env python3
"""
Static obstacle detection from AprilTag detections.

Uses TF2 to transform detected tag poses into the map frame, avoiding any
manual matrix arithmetic bugs. Chain:

    map → base_link          (from SLAM pose, broadcast by odom_ros2_zmq_sub)
    base_link → camera_link  (static 0.38 m forward, broadcast here)
    camera_link → tag_N      (dynamic per-detection, broadcast here)

TF2 lookup_transform('map', 'camera_link') + tag_in_camera = stable world pos.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from racecar_msgs.msg import AprilTagDetectionArray
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import message_filters
from scipy.spatial.transform import Rotation as Rot


class StaticObstacleDetector(Node):
    def __init__(self):
        super().__init__('static_obstacle_detection_node')

        self.declare_parameter('odom_topic', '/SLAM/Pose')
        self.declare_parameter('static_tag_topic', '/SLAM/Tag_Detections_Dynamic')
        self.declare_parameter('static_obs_size', 0.2)
        self.declare_parameter('static_obs_topic', '/Obstacles/Static')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        # Camera offset from rear axle — must match pose_offset in SLAM config.
        # Camera is 0.38 m forward of rear axle, no lateral offset, no rotation.
        self.declare_parameter('cam_x_offset', 0.38)

        odom_topic = self.get_parameter('odom_topic').value
        tag_topic = self.get_parameter('static_tag_topic').value
        self.static_obs_size = self.get_parameter('static_obs_size').value
        obs_topic = self.get_parameter('static_obs_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        cam_x = self.get_parameter('cam_x_offset').value

        # Static TF: base_link → camera_link (camera is cam_x m forward)
        self._static_tf_pub = StaticTransformBroadcaster(self)
        cam_tf = TransformStamped()
        cam_tf.header.stamp = self.get_clock().now().to_msg()
        cam_tf.header.frame_id = self.robot_frame
        cam_tf.child_frame_id = self.camera_frame
        cam_tf.transform.translation.x = cam_x
        cam_tf.transform.translation.y = 0.0
        cam_tf.transform.translation.z = 0.0
        cam_tf.transform.rotation.w = 1.0
        self._static_tf_pub.sendTransform(cam_tf)

        # TF2 buffer + listener to do map → camera lookup
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.obs_pub = self.create_publisher(MarkerArray, obs_topic, 1)

        # Tag is on top of obstacle cube (cube center is half-size below tag)
        self._half_size = self.static_obs_size / 2.0

        pose_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        tag_sub = message_filters.Subscriber(
            self, AprilTagDetectionArray, tag_topic
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, tag_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.detect_obs)

        self.get_logger().info(
            f"Obstacle detection ready. {odom_topic} + {tag_topic} → {obs_topic}"
        )

    def _tag_world_pos(self, tag, stamp):
        """
        Convert a tag detection (pose in camera_link frame) to map-frame
        position using TF2. Returns np.array([x,y,z]) or None on failure.
        """
        # Build tag pose in camera frame as a numpy 4x4
        q = [tag.pose.orientation.x, tag.pose.orientation.y,
             tag.pose.orientation.z, tag.pose.orientation.w]
        R = Rot.from_quat(q).as_matrix()
        T_tag2cam = np.eye(4)
        T_tag2cam[:3, :3] = R
        T_tag2cam[0, 3] = tag.pose.position.x
        T_tag2cam[1, 3] = tag.pose.position.y
        T_tag2cam[2, 3] = tag.pose.position.z

        # Obstacle center is half_size below tag in tag's z axis
        obs_offset = R @ np.array([0.0, 0.0, -self._half_size])
        tag_in_cam = np.array([tag.pose.position.x,
                               tag.pose.position.y,
                               tag.pose.position.z])
        obs_in_cam = tag_in_cam + obs_offset  # obstacle in camera_link frame

        try:
            # TF2: map → camera_link
            t = self._tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame,
                stamp, timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            try:
                # Fallback: latest available transform
                t = self._tf_buffer.lookup_transform(
                    self.map_frame, self.camera_frame,
                    rclpy.time.Time()
                )
            except Exception:
                return None

        # Apply map → camera transform to obs_in_cam
        tr = t.transform.translation
        rot = t.transform.rotation
        R_map2cam = Rot.from_quat(
            [rot.x, rot.y, rot.z, rot.w]
        ).as_matrix()

        obs_world = R_map2cam @ obs_in_cam + np.array([tr.x, tr.y, tr.z])
        return obs_world

    def detect_obs(self, odom_msg, tag_list):
        static_obs_msg = MarkerArray()
        stamp = odom_msg.header.stamp
        detected = []

        for tag in tag_list.detections:
            if tag.id in detected:
                continue
            detected.append(tag.id)

            world_pos = self._tag_world_pos(tag, stamp)
            if world_pos is None:
                continue

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = 'static_obs'
            marker.id = tag.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(world_pos[0])
            marker.pose.position.y = float(world_pos[1])
            marker.pose.position.z = float(world_pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 153.0 / 255.0
            marker.color.a = 0.8
            static_obs_msg.markers.append(marker)

        self.obs_pub.publish(static_obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
