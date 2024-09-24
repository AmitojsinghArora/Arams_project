#!/usr/bin/env python3

import random
from enum import Enum
from math import radians
import math
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String

# Map Bounds
map_min_x = -0.5
map_max_x = 12.5
map_min_y = -0.5
map_max_y = 12.5

# Calculate the centers of the 5x5 grid cells
grid_size = 5
cell_width = (map_max_x - map_min_x) / grid_size
cell_height = (map_max_y - map_min_y) / grid_size

grid_centers = [
    (map_min_x + (i + 0.5) * cell_width, map_min_y + (j + 0.5) * cell_height, 0.0)
    for i in range(grid_size) for j in range(grid_size)
]

# Shuffle the grid centers to visit them in random order
random.shuffle(grid_centers)

# Initialize an index to keep track of the current point
current_index = 0

success = True
initial_pose = None

class State(Enum):
    INIT = 1
    NAVIGATE = 2
    ROTATE = 3
    RETURN = 4

class ArtExplorer(Node):
    def __init__(self):
        super().__init__('auto_goals')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.subscription_apriltag = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.apriltag_callback,
            10
        )
        self.subscription_detected_images = self.create_subscription(
            String,
            '/detected_images',
            self.detected_images_callback,
            10
        )
        self.initial_pose = None
        self.detected_tags = []
        self.detected_images = []
        self.state = State.INIT

    def amcl_pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info(f"Initial pose: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}")

    def apriltag_callback(self, msg):
        for detection in msg.detections:
            if detection.decision_margin > 180 and detection.id not in self.detected_tags:
                self.detected_tags.append(detection.id)
                self.get_logger().info(f"Detected tag with ID: {detection.id} and decision margin: {detection.decision_margin}")
                if len(self.detected_tags) >= 3 and len(self.detected_images) >= 3:
                    self.state = State.RETURN

    def detected_images_callback(self, msg):
        detected_image = msg.data
        self.get_logger().info(f"Detected image: {detected_image}")

    def send_goal(self, position, orientation):
        """Create action goal object and send to action server, check if goal accepted"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation = orientation

        self.get_logger().info(f"Received new goal => X: {goal.pose.pose.position.x} Y: {goal.pose.pose.position.y}")

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        self.get_logger().info("Goal Accepted!")

        return goal_handle

    def check_result(self, goal_handle):
        """Check for task completion while blocking further execution"""
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached Goal!!!")
            self.state = State.ROTATE

        return status

    def perform_rotation(self):
        """Perform a complete 360-degree rotation"""
        for angle in range(0, 360, 90):  # Rotate in 90-degree increments
            quat = Quaternion()
            angle_radians = radians(angle)
            quat.w = round(math.cos(angle_radians / 2), 3)
            quat.x = 0.0
            quat.y = 0.0
            quat.z = round(math.sin(angle_radians / 2), 3)

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position = Point()  # Stay in the same position
            goal.pose.pose.orientation = quat

            send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Rotation goal was rejected")
                return

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)

            status = get_result_future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error("Rotation goal failed")
                return

        self.state = State.NAVIGATE

    def return_to_initial_pose(self):
        if self.initial_pose is None:
            self.get_logger().error("Initial pose is not set")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = self.initial_pose.position
        goal.pose.pose.orientation = self.initial_pose.orientation

        self.get_logger().info("Returning to initial pose")

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Return to initial pose goal was rejected")
            return

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Returned to initial pose")
            self.log_detections()
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().error("Failed to return to initial pose")

    def log_detections(self):
        """Log the detected AprilTags and YOLO detections"""
        for tag_id in self.detected_tags:
            for detected_image in self.detected_images_callback:
                if detected_image not in self.detected_images:
                    self.detected_images.append(detected_image)
                    self.get_logger().info(f"{tag_id}: {detected_image}")

def generate_next_position():
    """Return the next center point from the shuffled grid centers"""
    global current_index

    if current_index < len(grid_centers):
        position = Point()
        x, y, z = grid_centers[current_index]
        position.x = x
        position.y = y
        position.z = z
        current_index += 1
        return position
    else:
        return None

def generate_orientation():
    """Generate random orientation"""
    quat = Quaternion()
    quat.w = 1.0
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat

def main():
    rclpy.init()

    auto_explorer = ArtExplorer()

    # wait for action server to come up
    while not auto_explorer.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        auto_explorer.get_logger().info("Server still not available; waiting...")

    while rclpy.ok():
        if auto_explorer.state == State.INIT:
            if auto_explorer.initial_pose is not None:
                auto_explorer.get_logger().info("Initial position logged.")
                auto_explorer.state = State.NAVIGATE

        elif auto_explorer.state == State.NAVIGATE:
            position = generate_next_position()
            if position is not None:
                orientation = generate_orientation()
                goal_handle = auto_explorer.send_goal(position, orientation)
                status = auto_explorer.check_result(goal_handle)
            else:
                auto_explorer.state = State.RETURN

        elif auto_explorer.state == State.ROTATE:
            auto_explorer.perform_rotation()

        elif auto_explorer.state == State.RETURN:
            auto_explorer.return_to_initial_pose()
            break

        rclpy.spin_once(auto_explorer)

    auto_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()