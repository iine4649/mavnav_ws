#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class WarehouseScan(Node):
    def __init__(self):
        super().__init__('warehouse_scan')

        self.declare_parameter("height", 1.5)
        self.declare_parameter("x_min", 0.0)
        self.declare_parameter("x_max", 5.0)
        self.declare_parameter("y_min", 0.0)
        self.declare_parameter("y_max", 10.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("row_step", 1.0)

        self.h = float(self.get_parameter("height").value)
        self.frame = self.get_parameter("frame_id").value
        self.row_step = float(self.get_parameter("row_step").value)

        self.state = None
        self.cur_pose = None

        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_cb, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, 10
        )

        self.pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.path = self.make_lawnmower_path()
        self.idx = 0

        self.timer = self.create_timer(0.1, self.timer_cb)

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.cur_pose = msg

    def make_lawnmower_path(self):
        xs, ys = [], []

        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)
        y_min = float(self.get_parameter("y_min").value)
        y_max = float(self.get_parameter("y_max").value)

        y = y_min
        forward = True

        path = []
        while y <= y_max + 1e-6:
            if forward:
                xs = [x_min, x_max]
            else:
                xs = [x_max, x_min]

            for x in xs:
                p = PoseStamped()
                p.header.frame_id = self.frame
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = self.h
                p.pose.orientation.w = 1.0
                path.append(p)

            forward = not forward
            y += self.row_step

        return path

    def timer_cb(self):
        if self.state is None:
            return

        # OFFBOARD/GUIDED モードへ入れる
        if self.state.mode not in ("OFFBOARD", "GUIDED"):
            self.send_setpoint()  # モード切替前から送る必要あり
            if self.mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.mode_client.call_async(req)

        # ARM
        if not self.state.armed and self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)

        # setpoint を送る
        self.send_setpoint()
        self.advance()

    def send_setpoint(self):
        target = self.path[self.idx]
        target.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(target)

    def advance(self):
        if self.cur_pose is None:
            return

        goal = self.path[self.idx]
        dx = self.cur_pose.pose.position.x - goal.pose.position.x
        dy = self.cur_pose.pose.position.y - goal.pose.position.y
        dz = self.cur_pose.pose.position.z - goal.pose.position.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < 0.2 and self.idx < len(self.path) - 1:
            self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
