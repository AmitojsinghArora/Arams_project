#!/usr/bin/env python3

import math
from math import sqrt, radians
import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import random
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion

# Map Bounds
map_min_x = -0.5
map_max_x = 12.5
map_min_y = -0.5
map_max_y = 12.5

success = True
initial_position = None

def main():
  global auto_chaos
  global nav_to_pose_client

  status = 4

  rclpy.init()
  auto_chaos = rclpy.create_node('auto_goals')
  # create Action Client object with desired message type and action name
  nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')
  
  # wait for action server to come up

  while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
    print("Server still not available; waiting...")

  

  while rclpy.ok():
    try:
      position = generatePosition()
      orientation = generateOrientation()
      goal_handle = sendGoal(position, orientation)
      status = checkResult(goal_handle)

    except KeyboardInterrupt:
      print("Shutdown requested... complying...")
      break

  nav_to_pose_client.destroy()
  auto_chaos.destroy_node()
  rclpy.shutdown()

def amcl_pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info(f"Initial pose: x={self.initial_pose.position.x}, y={self.initial_pose.position.y}, z={self.initial_pose.position.z}")

def sendGoal(position, orientation):

  """Create action goal object and send to action server, check if goal accepted"""

  global auto_chaos
  global nav_to_pose_client

  goal = NavigateToPose.Goal()
  goal.pose.header.frame_id = "map"
  goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
  goal.pose.pose.position = position
  goal.pose.pose.orientation = orientation

  print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))

  send_goal_future = nav_to_pose_client.send_goal_async(goal)
  rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

  goal_handle = send_goal_future.result()

  if not goal_handle.accepted:
    print("Goal was rejected")
    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

  print("Goal Accepted!")

  return goal_handle


def checkResult(goal_handle):

  """Check for task completion while blocking further execution"""

  get_result_future = goal_handle.get_result_async()

  rclpy.spin_until_future_complete(auto_chaos, get_result_future)

  status = get_result_future.result().status

  if status == GoalStatus.STATUS_SUCCEEDED:
    print("Reached Goal!!!")
    performRotation()

  return status

def performRotation():
    """Perform a complete 360-degree rotation"""
    global auto_chaos
    global nav_to_pose_client

    for angle in range(0, 360, 45):  # Rotate in 45-degree increments
        quat = Quaternion()
        angle_radians = radians(angle)
        quat.w = round(math.cos(angle_radians / 2), 3)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = round(math.sin(angle_radians / 2), 3)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
        goal.pose.pose.position = Point()  # Stay in the same position
        goal.pose.pose.orientation = quat

        send_goal_future = nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("Rotation goal was rejected")
            return

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(auto_chaos, get_result_future)

        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print("Rotation goal failed")
            return

    print("Completed 360-degree rotation")

# List of 15 Key points
key_points = [
    (2.7, 0.0, 0.0),
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

def generatePosition():
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

# def generatePosition():

#   """Randomize a pair of values to denote xy position on map"""

#   position = Point()

#   position.x = round(random.uniform(map_min_x,map_max_x), 2)

#   position.y = round(random.uniform(map_min_y,map_max_y), 2)

#   position.z = 0.0

#   return position


def generateOrientation():

  """Making the robot rotate"""

  quat = Quaternion()

  quat.w = round(random.uniform(-1.0,1.0),3)

  quat.x = 0.0

  quat.y = 0.0

  quat.z = sqrt(1 - quat.w*quat.w)

  return quat


if __name__ == '__main__':

    main()
