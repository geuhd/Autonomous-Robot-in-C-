from launch import LaunchDescription
from launch_ros.actions import Node

# Runs ON THE PI. Everything here talks directly to hardware (GPIO/I2C/serial/
# USB), which is exactly the stuff that has to run on the robot itself.
# Nothing here needs a display, so this is safe to run headless over SSH.
#
# The nav/SLAM/localization side (odom_publisher, slam_toolbox, costmap,
# path_planner, drive controller, any rqt/RViz GUI tools) all run on the more
# powerful PC/VM instead -- ROS 2 nodes discover each other automatically over
# DDS as long as both machines are on the same LAN with the same
# ROS_DOMAIN_ID, so there's no need for everything to live on one machine like
# ROS 1's single-roscore model required.
#
# ros2 launch practical_nav pi_hardware.launch.py

def generate_launch_description():
    return LaunchDescription([
        # base_link -> laser static transform: physical mount, lives with the
        # hardware side regardless of which machine is doing the SLAM/nav math.
        # EDIT to match how your LIDAR is actually mounted.
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
        # TODO: swap this in for your actual ldlidar_ros2 node -- package,
        # executable, and any params/remaps (topic/frame_id) depend on which
        # LD-Robot LIDAR model + driver fork you installed. Tell me what
        # `find ~/ros2_ws/src/ldlidar_ros2 -name "*.launch.py"` and its
        # package.xml <name> show and I'll fill this in for real.
        # Node(
        #     package="ldlidar_ros2",
        #     executable="ldlidar_publisher",
        #     name="ldlidar_publisher",
        #     output="screen",
        # ),
    ])
