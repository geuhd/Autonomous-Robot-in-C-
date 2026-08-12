from launch import LaunchDescription
from launch_ros.actions import Node

# Chapter 21, step 6.1: basic_manual_steer.launch
# The mapping launch file: drive around slowly with rqt_robot_steering while
# slam_toolbox (run separately, see below) builds the map from odometry + the
# laser scan. No path planner or costmap here -- we're just mapping.
#
# The static base_link -> laser transform values below (x=.1, roll=0,
# pitch=pi, yaw=pi) are the book's own numbers for its RPLidar A1 mount --
# EDIT these to match how your LIDAR is actually mounted.
#
# slam_toolbox publishes the map -> odom transform itself while it's running,
# so there is NO map -> odom static transform in this file (adding one would
# mean two publishers fighting over the same transform, which is exactly what
# the book warns against with roswtf -- ros2 doctor is the closest ROS 2
# equivalent for catching that).
#
# Run this, then in another terminal:
#   ros2 launch slam_toolbox online_async_launch.py
#
# ros2 launch practical_nav basic_manual_steer.launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=[
                "--x", ".1", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "3.1415", "--yaw", "3.1415",
                "--frame-id", "base_link", "--child-frame-id", "laser",
            ],
        ),
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
        # TODO: add your LIDAR driver node here, e.g. sllidar_ros2's
        # sllidar_node -- remap its scan/frame_id if they aren't already
        # "scan"/"laser".
        Node(
            package="rqt_robot_steering",
            executable="rqt_robot_steering",
            name="rqt_robot_steering",
            output="screen",
        ),
    ])
