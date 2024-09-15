#!/usr/bin/env python3

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

# List of 16 Key points
key_points = [
    (2.7, 0.0, 0.0),
    # (0.0, 1.0, 0.0),
    (2.5, 4.7, 0.0),
    (0.0, 12.0, 0.0),
    (3.0, 9.0, 0.0),
    (9.0, 9.0, 0.0),
    (9.0, 6.5, 0.0),
    (3.5, 12.0, 0.0),
    (9.0, 12.0, 0.0),
    (12.0, 12.0, 0.0),
    (12.0, 8.0, 0.0),
    (12.0, 0.5, 0.0),
    (9.0, 0.5, 0.0),
    (6.0, 0.0, 0.0),
    (6.0, 3.5, 0.0),
    (6.0, 5.5, 0.0)
]
# Initialize an index to keep track of the current point
current_index = 0

success = True
initial_pose = None

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

    def amcl_pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info(f"Initial pose: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}")

    def apriltag_callback(self, msg):
        for detection in msg.detections:
            if detection.decision_margin > 150:
                self.detected_tags.append(detection.id)
                self.get_logger().info(f"Detected tag with ID: {detection.id} and decision margin: {detection.decision_margin}")
                if len(self.detected_tags) >= 3:
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

                    return

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
            self.perform_rotation()

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

    def log_detections(self):
        """Log the detected AprilTags and YOLO detections"""
        for tag_id in self.detected_tags:
            for detected_image in self.detected_images_callback:
                self.get_logger().info(f"{tag_id}: {detected_image}")

def generate_position():
    """Return specific points to denote xy position on map"""
    global current_index

    position = Point()

    # Get the current point from the list
    x, y, z = key_points[current_index]

    position.x = x
    position.y = y
    position.z = z

    # Update the index to the next point
    current_index = (current_index + 1) % len(key_points)

    return position

def generate_orientation():
    """Generate random orientation"""
    quat = Quaternion()
    quat.w = 1.0
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat



def main():
    global current_index

    rclpy.init()

    auto_explorer = ArtExplorer()

    # wait for action server to come up
    while not auto_explorer.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        auto_explorer.get_logger().info("Server still not available; waiting...")

    while rclpy.ok() and len(auto_explorer.detected_tags) < 3:
        try:
            position = generate_position()
            orientation = generate_orientation()
            goal_handle = auto_explorer.send_goal(position, orientation)
            status = auto_explorer.check_result(goal_handle)
        except KeyboardInterrupt:
            auto_explorer.get_logger().info("Shutdown requested... complying...")
            break

    auto_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()