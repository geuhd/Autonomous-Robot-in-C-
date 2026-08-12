from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
# rgpiod is started here too, since tick_publisher/simple_diff_drive/
# imu_publisher all connect to it and die immediately if it isn't already
# running (that's the "Failed to connect to rgpiod Daemon" error). This
# requires passwordless sudo for rgpiod specifically -- one-time setup:
#   sudo visudo
# then add a line (swap in your actual username and `which rgpiod` path):
#   prince ALL=(ALL) NOPASSWD: /usr/local/bin/rgpiod
# Without that, this launch file will just hang waiting for a sudo password
# prompt it can never show you. The more robust long-term fix is a systemd
# service (`sudo systemctl enable rgpiod`) so rgpiod is always running
# already and this launch file doesn't need to manage it at all -- worth
# switching to once you're past active development.
#
# ros2 launch practical_nav pi_hardware.launch.py

def generate_launch_description():
    # base_link -> laser static transform: physical mount, lives with the
    # hardware side regardless of which machine is doing the SLAM/nav math.
    # EDIT to match how your LIDAR is actually mounted.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        arguments=[
            "--x", ".1", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "3.1415", "--yaw", "3.1415",
            "--frame-id", "base_link", "--child-frame-id", "laser",
        ],
    )

    tick_publisher = Node(
        package="practical_hardware",
        executable="tick_publisher",
        name="tick_publisher",
        output="screen",
    )

    simple_diff_drive = Node(
        package="practical",
        executable="simple_diff_drive",
        name="simple_diff_drive",
        output="screen",
    )

    imu_publisher = Node(
        package="practical_hardware",
        executable="imu_publisher",
        name="imu_publisher",
        output="screen",
    )

    # TODO: swap this in for your actual ldlidar_ros2 node -- package,
    # executable, and any params/remaps (topic/frame_id) depend on which
    # LD-Robot LIDAR model + driver fork you installed. Tell me what
    # `find ~/ros2_ws/src/ldlidar_ros2 -name "*.launch.py"` and its
    # package.xml <name> show and I'll fill this in for real.
    # lidar_driver = Node(
    #     package="ldlidar_ros2",
    #     executable="ldlidar_publisher",
    #     name="ldlidar_publisher",
    #     output="screen",
    # )

    return LaunchDescription([
        # start the rgpiod daemon first...
        ExecuteProcess(
            cmd=["sudo", "rgpiod"],
            name="rgpiod",
            output="screen",
        ),
        # ...and give it a moment to come up before anything tries to
        # connect to it. 2 seconds is generous; rgpiod starts almost
        # instantly, but this costs nothing and avoids a race on a loaded Pi.
        TimerAction(
            period=2.0,
            actions=[
                static_tf,
                tick_publisher,
                simple_diff_drive,
                imu_publisher,
                # lidar_driver,
            ],
        ),
    ])
