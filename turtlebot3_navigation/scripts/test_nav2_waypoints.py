# navigating to specific goal destinations using nav2

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_to_pose import Nav2ToPose

destinations = {
    "destination_1": [-0.0480, -1.7939],
    "destination_2": [1.3429, -1.5691],
    "destination_3": [-0.0572, -0.5580],
    "destination_4": [-0.0298, 1.8361],
    "destination_5": [-2.0493, -0.5635],
}


def main(args=None):

    rclpy.init(args=args)

    navigator = Nav2ToPose()

    poses = []

    # converting the destinations to PoseStamped objects
    for key in destinations:
        destinations[key] = navigator.list_to_posestamped(destinations[key])
        poses.append(destinations[key])

    # sending the robot to follow the waypoints
    navigator.follow_waypoints(poses)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
