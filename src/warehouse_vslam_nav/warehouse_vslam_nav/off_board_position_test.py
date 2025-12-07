#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class OffboardPositionTest(Node):
    def __init__(self):
        super().__init__('offboard_position_test')
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        self.timer = self.create_timer(0.05, self.timer_cb)  # 20Hz

    def timer_cb(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 原点付近にちょっとだけオフセットした setpoint
        msg.pose.position.x = 1.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.0
        # 姿勢はとりあえずそのまま（PX4が補完）
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OffboardPositionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
