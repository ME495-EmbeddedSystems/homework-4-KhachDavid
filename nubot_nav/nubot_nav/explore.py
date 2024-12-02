import random

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def explore_entry(args=None):
    """Entry point for the explore node."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


class Explore(Node):
    """Explore the navigation stack with a state machine."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('explore')
        self.get_logger().info('Exploring the navigation stack')

        # Use the map_saver_cli in the nav2_map_server package to save the resulting map
        self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Create an action client for the NavigateToPose action
        self.goal_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.previous_feedback = None
        self.map_data = None
        self.target_positions = {}
        self.current_state = None

        self.goal_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server is available')

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Handle the map message."""
        self.map_data = msg
        # Initialize target positions (corners of the map) only once
        if not self.target_positions:
            self.initialize_target_positions()

        # Process state machine logic based on current state
        self.handle_state()

    def initialize_target_positions(self):
        """Initialize the target positions (corners of the map)."""
        if self.map_data is None:
            self.get_logger().error('No map data available')
            return

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        # random list of x and y coordinates in the right range
        num_points = 200
        x = np.random.uniform(0, width * resolution, num_points)
        y = np.random.uniform(0, height * resolution, num_points)
        self.target_positions = {(x[i], y[i]) for i in range(num_points)}

    def handle_state(self):
        """Handle the current state in the state machine."""
        if self.map_data is None:
            return

        # Move to the current target corner
        target_position = random.choice(list(self.target_positions))

        self.navigate_to_frontier(target_position)

    def navigate_to_frontier(self, frontier):
        """Send the robot to the next target position (corner)."""
        if not self.goal_client.wait_for_server(timeout_sec=10):
            self.get_logger().error('Navigation server not available')
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = frontier[0]
        goal_pose.pose.position.y = frontier[1]
        goal_pose.pose.orientation.w = 1.0  # No rotation

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Navigating to target at {frontier}')
        future = self.goal_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_goal_feedback
        )
        future.add_done_callback(self.on_goal_result)

    def on_goal_feedback(self, feedback):
        """Handle the feedback of the navigation action."""
        self.get_logger().info(f'Feedback: {feedback}')
        # save reference to feedback for future use
        # if estimation of time to goal is not changing, then cancel the goal and send a new one
        # if the robot is stuck, then cancel the goal and send a new one
        if feedback.feedback.distance_remaining < 0.1:
            target_position = random.choice(list(self.target_positions))
            self.navigate_to_frontier(target_position)

    def on_goal_result(self, future):
        """Handle the result of the navigation action."""
        # choose a random target position
        target_position = random.choice(list(self.target_positions))
        self.navigate_to_frontier(target_position)


if __name__ == '__main__':
    import sys
    explore_entry(args=sys.argv)
