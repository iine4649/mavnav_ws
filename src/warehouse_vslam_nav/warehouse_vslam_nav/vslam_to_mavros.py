#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VslamToMavrosBridge(Node):
    def __init__(self):
        super().__init__("vslam_to_mavros_bridge")

        self.declare_parameter("vslam_odom_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("vision_frame_id", "odom")

        vslam_topic = self.get_parameter("vslam_odom_topic").value
        vision_topic = self.get_parameter("vision_pose_topic").value
        self.frame_id = self.get_parameter("vision_frame_id").value

        self.sub = self.create_subscription(
            Odometry, vslam_topic, self.odom_cb, 10
        )
        self.pub = self.create_publisher(
            PoseStamped, vision_topic, 10
        )

        self.get_logger().info(f"bridge: {vslam_topic} → {vision_topic}")

    def odom_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = VslamToMavrosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
