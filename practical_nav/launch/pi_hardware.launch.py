import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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
    # NOTE: there is deliberately no custom base_link -> laser static
    # transform here anymore. ldlidar_ros2's own ld19.launch.py (included
    # below) already publishes base_link -> base_laser itself (confirmed by
    # its own log output: "Spinning until stopped - publishing transform...
    # from 'base_link' to 'base_laser'"), which is the frame_id its
    # LaserScan messages actually use. A second, separately-defined
    # transform to a "laser" frame that nothing else references would just
    # be dead weight -- slam_toolbox reads the frame_id straight off the
    # incoming scan message and looks up whatever transform already
    # connects it to base_link, so the driver's own transform is sufficient.
    # If you re-mount the LIDAR differently than its default (translation
    # 0, 0, 0.18, no rotation), edit ldlidar_ros2's own launch file/params
    # rather than adding a second, conflicting transform here.

    tick_publisher = Node(
        package="practical_hardware",
        executable="tick_publisher",
        name="tick_publisher",
        output="screen",
    )

    simple_diff_drive = Node(
        package="practical_hardware",
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

    # LDROBOT D500 (STL-19P) via the ldrobotSensorTeam/ldlidar_ros2 package.
    # Rather than reimplement its Node() call here (and risk getting the
    # internal executable name/params wrong), we just include the driver's
    # own launch file -- it's the maintained, known-good way to start it.
    # EDIT the port if `ls /dev/ttyUSB*` shows something other than
    # /dev/ttyUSB0 -- either edit it here via launch_arguments, or edit
    # port_name directly in ldlidar_ros2's own ld19.launch.py.
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ldlidar_ros2"),
                "launch",
                "ld19.launch.py",
            )
        ),
    )

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
                tick_publisher,
                simple_diff_drive,
                imu_publisher,
                lidar_driver,
            ],
        ),
    ])
