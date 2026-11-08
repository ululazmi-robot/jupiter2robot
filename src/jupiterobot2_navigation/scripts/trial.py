#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import math

def create_pose(nav, x, y, yaw_deg=0.0):
    """
    Helper function to create a PoseStamped message.
    yaw_deg: The direction the robot should face (in degrees).
    0 = East/Forward, 90 = North/Left, 180 = West/Backward, 270 = South/Right.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    
    # Convert degrees to radians
    yaw_rad = math.radians(yaw_deg)
    
    # Convert Euler angle (yaw) to Quaternion (z and w)
    # This is a standard formula for 2D rotation
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Set Initial Pose (Home)
    # Ensure this matches your robot's starting position in Gazebo
    initial_pose = create_pose(nav, 0.0, 0.0, yaw_deg=0.0)
    nav.setInitialPose(initial_pose)

    # 2. Wait for Navigation Stack to be fully ready
    print("Waiting for Nav2 to become active...")
    nav.waitUntilNav2Active()

    # 3. Define our Locations
    # format: create_pose(navigator, x, y, orientation_in_degrees)
    home    = create_pose(nav, -0.176, -0.018, yaw_deg=0.0)
    point_a = create_pose(nav, -0.974, 1.0, yaw_deg=90.0)  # Faces "North" at Point A
    point_b = create_pose(nav, -0.932, 3.69, yaw_deg=180.0) # Faces "West" at Point B

    # 4. Define the Mission Sequence
    mission_plan = [point_a, home, point_b, home]
    mission_names = ["Point A", "Home", "Point B", "Home"]

    print("--- Starting Mission: Home -> A -> Home -> B -> Home ---")

    # 5. Execute the Mission
    for i, goal in enumerate(mission_plan):
        print(f"\n[Leg {i+1}] Moving to: {mission_names[i]}")
        nav.goToPose(goal)

        # While the robot is moving...
        while not nav.isTaskComplete():
            # (Optional) You can print feedback here, like distance to goal
            feedback = nav.getFeedback()
            if feedback and i % 10 == 0: # Print every 10th feedback loop
                 print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")

        # Check the result of the movement
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"SUCCESS: Reached {mission_names[i]}. Resting for 2 seconds.")
            time.sleep(2.0) 
        elif result == TaskResult.CANCELED:
            print(f"CANCELLED: Mission was canceled at {mission_names[i]}.")
            break
        elif result == TaskResult.FAILED:
            print(f"FAILED: Could not reach {mission_names[i]}. Check for obstacles!")
            break

    print("\n--- All points reached. Mission Complete! ---")
    rclpy.shutdown()

if __name__ == '__main__':
    main()