#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Set our "Home" or Initial Pose (where the robot is now)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)

    # 2. Wait for Nav2 to be active
    nav.waitUntilNav2Active()

    # 3. Define Point A (Hardcoded coordinates)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = -0.974  # Replace with your actual X
    goal_pose.pose.position.y = 1.0 # Replace with your actual Y
    goal_pose.pose.orientation.w = 0.00412

    # 4. Move to the point
    print("Moving to Point A...")
    nav.goToPose(goal_pose)

    # 5. Check progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # Optional: print distance remaining
        # print(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    print("Result: " + str(nav.getResult()))
    rclpy.shutdown()

if __name__ == '__main__':
    main()