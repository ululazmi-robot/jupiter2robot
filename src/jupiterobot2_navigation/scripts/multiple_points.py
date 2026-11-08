#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(nav, x, y, w=1.0):
    """Helper function to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Set Initial Pose (Home)
    # Ensure these coordinates match where the robot starts in Gazebo
    initial_pose = create_pose(nav, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # 2. Wait for Nav2
    nav.waitUntilNav2Active()

    # 3. Define our Locations (Update these with your RViz coordinates!)
    home    = create_pose(nav, -0.176, -0.018)
    point_a = create_pose(nav, -0.974, 1.0)
    point_b = create_pose(nav, -0.932, 3.69)

    # 4. Define the Mission Sequence
    mission_plan = [point_a, home, point_b, home]
    mission_names = ["Point A", "Home", "Point B", "Home"]

    # 5. Execute the Mission
    for i, goal in enumerate(mission_plan):
        print(f"--- Starting Leg {i+1}: Moving to {mission_names[i]} ---")
        nav.goToPose(goal)

        while not nav.isTaskComplete():
            # You can add feedback here if you want to see distance remaining
            pass

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Reached {mission_names[i]} successfully!")
            time.sleep(2) # Pause for 2 seconds at each stop
        else:
            print(f"Failed to reach {mission_names[i]}. Result code: {result}")
            break

    print("Mission Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()