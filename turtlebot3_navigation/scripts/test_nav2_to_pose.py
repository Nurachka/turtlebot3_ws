"""
Testing the nav2 to pose script
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_to_pose import Nav2ToPose


def main(args=None):

    rclpy.init(args=args)

    navigation_node = Nav2ToPose()

    # define a goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigation_node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # send the robot to the goal pose
    navigation_node.go_to_pose(goal_pose)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
