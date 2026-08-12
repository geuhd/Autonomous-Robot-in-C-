from launch import LaunchDescription
from launch_ros.actions import Node

# Runs ON THE PC/VM. Pairs with pi_hardware.launch.py running on the Pi.
# VM side of the old "basic_manual_waypoint" step: odometry fusion + a
# single-waypoint test, no path planning/obstacle avoidance yet.
#
# rqt_robot_steering is deliberately not included here (same reasoning as
# before): it and simple_drive_controller would both publish cmd_vel and
# fight each other.
#
# ros2 launch practical_nav vm_manual_waypoint.launch.py

def generate_launch_description():
    return LaunchDescription([
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
        # simple_drive_controller.cpp listens on "waypoint_2d" (normally fed
        # by path_planner.cpp), but there's no path planner running yet here
        # -- remap it to "goal_2d", what manual_pose_and_goal_pub publishes.
        Node(
            package="practical",
            executable="simple_drive_controller",
            name="simple_drive_controller",
            output="screen",
            remappings=[("waypoint_2d", "goal_2d")],
        ),
    ])
