from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Chapter 21, step 8.2: basic_full.launch
# Full autonomy: serves the saved map, locks map -> odom together (we're not
# running SLAM here, so nothing else publishes that transform -- see the
# book's own note that gmapping normally provides it), runs the costmap and
# the A* path planner, and drives to whatever goal_2d receives.
#
# ROS 1 -> ROS 2 differences worth knowing before you run this:
#   * map_server and nav2_costmap_2d are both LIFECYCLE nodes in ROS 2 -- they
#     start up "unconfigured" and need a lifecycle_manager to bring them to
#     "active", unlike ROS 1's map_server which just ran.
#   * nav2_costmap_2d publishes its costmap under <name>/costmap/costmap, not
#     the bare "costmap" topic the ROS 1 costmap_2d package used -- hence the
#     remap on path_planner below.
#   * manual_pose_and_goal_pub is NOT included here (too much terminal output
#     mixed with everything else per the book) -- run it in its own terminal,
#     or use RViz's own goal tool once you've bridged it to goal_2d.
#
# ros2 launch practical_nav basic_full.launch.py map_file:=/path/to/myFirstMap.yaml

def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("practical_nav"), "maps", "myFirstMap.yaml"]
        ),
        description="Full path to the map yaml file saved with nav2_map_server's map_saver_cli",
    )
    costmap_params = PathJoinSubstitution(
        [FindPackageShare("practical_nav"), "param", "costmap_basic.yaml"]
    )

    return LaunchDescription([
        map_file_arg,

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
        # locks map and odom together since nothing else (no SLAM node) is
        # running to publish this transform for us
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
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
        # TODO: add your LIDAR driver node here (e.g. sllidar_ros2).

        # map_server: lifecycle node, needs a lifecycle_manager to activate it
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": LaunchConfiguration("map_file")}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["map_server"]}],
        ),

        # costmap: also a lifecycle node -- name/namespace "costmap" here
        # matches costmap_basic.yaml's costmap: costmap: ros__parameters: nesting
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="costmap",
            namespace="costmap",
            output="screen",
            parameters=[costmap_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["costmap/costmap"]}],
        ),

        # path_planner.cpp subscribes to the bare topic "costmap" -- remap it
        # to where nav2_costmap_2d actually publishes
        Node(
            package="practical_nav",
            executable="path_planner",
            name="path_planner",
            output="screen",
            remappings=[("costmap", "/costmap/costmap")],
        ),
        # no remap here (unlike basic_manual_waypoint.launch.py) -- this time
        # simple_drive_controller listens on the real "waypoint_2d" the path
        # planner publishes
        Node(
            package="practical",
            executable="simple_drive_controller",
            name="simple_drive_controller",
            output="screen",
        ),
    ])
