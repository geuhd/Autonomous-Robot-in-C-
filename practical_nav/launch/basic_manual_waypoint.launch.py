from launch import LaunchDescription
from launch_ros.actions import Node

# Chapter 21, step 5.2: basic_manual_waypoint.launch
# Adds odometry + a drive controller so the robot can be sent to a single
# waypoint published by manual_pose_and_goal_pub -- no obstacle avoidance or
# path planning yet, just "turn to face it, then drive straight there".
#
# rqt_robot_steering is deliberately left out here: it and
# simple_drive_controller would both be publishing cmd_vel at the same time
# and fight each other, same reasoning the book gives in this step.
#
# ros2 launch practical_nav basic_manual_waypoint.launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="practical_hardware",
            executable="tick_publisher",
            name="tick_publisher",
            output="screen",
        ),
        Node(
            package="practical",
            executable="simple_diff_drive",
            name="simple_diff_drive",
            output="screen",
        ),
        Node(
            package="practical_hardware",
            executable="imu_publisher",
            name="imu_publisher",
            output="screen",
        ),
        Node(
            package="practical_localization",
            executable="odom_publisher",
            name="odom_publisher",
            output="screen",
        ),
        Node(
            package="practical_localization",
            executable="manual_pose_and_goal_pub",
            name="manual_pose_and_goal_pub",
            output="screen",
        ),
        # simple_drive_controller.cpp listens on "waypoint_2d" (that's what
        # path_planner.cpp normally feeds it), but we don't have a path
        # planner running yet -- so remap it to listen straight to "goal_2d",
        # the topic manual_pose_and_goal_pub actually publishes.
        Node(
            package="practical",
            executable="simple_drive_controller",
            name="simple_drive_controller",
            output="screen",
            remappings=[("waypoint_2d", "goal_2d")],
        ),
    ])
