#!/usr/bin/env python3
"""
ROS 2 node for navigating to a goal pose.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


class Nav2ToPose(Node):
    """This class navigates to a goal pose"""

    def __init__(self):
        """Constructor."""
        # Initialize the class using the constructor
        super().__init__("nav2_to_pose")

        # Launching the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()

        # Waiting for navigation to fully activate
        self.navigator.waitUntilNav2Active()

    def list_to_posestamped(self, destinations: list) -> PoseStamped:
        """Convert a list of [x, y] to a PoseStamped object.
        Args:
            position_list (list): A list containing x and y coordinates.
        Returns:
            PoseStamped: The corresponding PoseStamped object.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = destinations[0]
        pose.pose.position.y = destinations[1]
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def go_to_pose(self, pose: PoseStamped):
        """Go to goal pose.
        Args:
            pose (PoseStamped): The target pose to navigate to.
        """

        # Clearing all costmaps before sending to a goal
        self.navigator.clearAllCostmaps()

        # Sending the robot to the goal pose
        self.navigator.goToPose(pose)

        # While robot is moving to the goal pose, get the feedback
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Print the estimated time of arrival in seconds
                eta_sec = (
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                print(f"Estimated time of arrival: {eta_sec:.0f} seconds")

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print("Goal pose reached successfully!")
        elif result == TaskResult.CANCELED:
            print("Goal pose navigation was canceled!")
        elif result == TaskResult.FAILED:
            print("Goal pose navigation failed!")

    def follow_waypoints(self, poses: list):
        """Follow waypoints.
        Args:
            poses (list): List of PoseStamped objects representing waypoints.
        """
        # Clearing all costmaps before sending to a goal
        self.navigator.clearAllCostmaps()

        # Following the poses in sequence
        self.navigator.followWaypoints(poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(f"Currently going to waypoint: {feedback.current_waypoint+1}")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("All waypoints reached successfully!")
        elif result == TaskResult.CANCELED:
            print("Waypoint navigation was canceled!")
        elif result == TaskResult.FAILED:
            print("Waypoint navigation failed!")
