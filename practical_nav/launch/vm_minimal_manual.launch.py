from launch import LaunchDescription
from launch_ros.actions import Node

# Runs ON THE PC/VM. Pairs with pi_hardware.launch.py running on the Pi.
# This is the VM side of the old "minimal_manual" step: just the steering GUI
# (rqt_robot_steering needs a display, which the headless Pi doesn't have).
# It publishes cmd_vel over the network; simple_diff_drive on the Pi picks it
# up the same way it would if everything were on one machine.
#
# ros2 launch practical_nav vm_minimal_manual.launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rqt_robot_steering",
            executable="rqt_robot_steering",
            name="rqt_robot_steering",
            output="screen",
        ),
    ])
