from launch import LaunchDescription
from launch_ros.actions import Node

# Chapter 21, step 3.3: minimal_manual.launch
# Brings up just enough to drive the robot around by hand with rqt_robot_steering:
# encoder ticks + the motor controller + a steering GUI that publishes cmd_vel.
#
# ros2 launch practical_nav minimal_manual.launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="practical_hardware",
            executable="tick_publisher",
            name="tick_publisher",
            output="screen",
        ),
        # NOTE: practical/CMakeLists.txt currently names this target
        # "simple_diff_drive" but builds it from src/simple_diff.cpp -- fix
        # the add_executable() source path (or rename the file) before this
        # will compile. See the coding-style summary for details.
        Node(
            package="practical",
            executable="simple_diff_drive",
            name="simple_diff_drive",
            output="screen",
        ),
        Node(
            package="rqt_robot_steering",
            executable="rqt_robot_steering",
            name="rqt_robot_steering",
            output="screen",
        ),
    ])
