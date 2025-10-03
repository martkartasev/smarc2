#!/usr/bin/python3

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from sam_diving_controller.ActionServerDiveSub import HydropointServer
from sam_diving_controller.ParamUtils import DivingModelParam
from sam_diving_controller.SAMDivePub import SAMDivePub
from sam_diving_controller.controllers.DiveControllerONNX import DiveControllerONNX
from smarc_action_base.smarc_action_base import (
    ActionType,
)
from smarc_msgs.action import BaseAction
from smarc_msgs.msg import Topics as SMaRCTopics


def rl_waypoint_following():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("ActionServerDivingNode")

    node.declare_parameter('dive_pub_rate', 0.1)
    node.declare_parameter('dive_controller_rate', 0.1)
    node.declare_parameter('dive_sub_rate', 0.1)

    # This is not a frequency, but a period.
    # t = 10 -> callback gets called every 10 sec
    dive_pub_rate = node.get_parameter('dive_pub_rate').get_parameter_value().double_value
    dive_controller_rate = node.get_parameter('dive_controller_rate').get_parameter_value().double_value
    dive_sub_rate = node.get_parameter('dive_sub_rate').get_parameter_value().double_value

    param = DivingModelParam(node).get_param()
    action_type = ActionType(BaseAction)
    heartbeat_topic = SMaRCTopics.WARA_PS_ACTION_SERVER_HB_TOPIC

    dive_sub = HydropointServer(node, "go_to_hydropoint", action_type, param, heartbeat_topic)
    dive_pub = SAMDivePub(node, dive_sub, param)
    dive_controller = DiveControllerONNX(node, dive_pub, dive_sub, param, dive_controller_rate)

    node.create_timer(dive_pub_rate, dive_pub.update)
    node.create_timer(dive_controller_rate, dive_controller.update)
    node.create_timer(dive_sub_rate, dive_sub.update)

    def _loginfo(node, s):
        node.get_logger().info(s)

    _loginfo(node, "Action Server")
    _loginfo(node, "Created MVC")

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
        # rclpy.spin(node)
        _loginfo(node, "Spinning up")
    except KeyboardInterrupt:
        pass

    _loginfo(node, "Shutting down")


if __name__ == "__main__":
    rl_waypoint_following()
