from launch import LaunchDescription
from launch_ros.actions import Node

# Runs ON THE PC/VM. Pairs with pi_hardware.launch.py running on the Pi
# (which already publishes the base_link -> laser static transform and the
# actual LIDAR scan).
#
# VM side of the old "basic_manual_steer" mapping step: odometry + a steering
# GUI to drive slowly while mapping. slam_toolbox publishes the map -> odom
# transform itself while it's running, so run it alongside this in a second
# terminal on the VM:
#
#   ros2 launch slam_toolbox online_async_launch.py
#
# ros2 launch practical_nav vm_mapping.launch.py

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
        Node(
            package="rqt_robot_steering",
            executable="rqt_robot_steering",
            name="rqt_robot_steering",
            output="screen",
        ),
    ])
