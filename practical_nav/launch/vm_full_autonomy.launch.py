from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Runs ON THE PC/VM. Pairs with pi_hardware.launch.py running on the Pi
# (which handles the base_link -> laser transform, encoders, motor control,
# IMU, and the LIDAR itself).
#
# VM side of the old "basic_full" step: full autonomy. Serves the saved map,
# locks map -> odom together (nothing else is publishing that transform since
# we're not running SLAM live during an autonomous run), runs the costmap and
# A* path planner, and drives to whatever goal_2d receives.
#
# Why this split makes sense: map_server, costmap, and the path planner are
# all CPU-hungry compared to the simple GPIO/I2C reads the Pi is doing, and
# none of them touch hardware -- ROS 2's DDS discovery means they don't need
# to be on the same machine as the sensors they're consuming, unlike ROS 1's
# single-roscore model.
#
# ros2 launch practical_nav vm_full_autonomy.launch.py map_file:=/path/to/myFirstMap.yaml

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
            package="practical_localization",
            executable="odom_publisher",
            name="odom_publisher",
            output="screen",
        ),

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
        # no remap here -- simple_drive_controller listens on the real
        # "waypoint_2d" the path planner publishes
        Node(
            package="practical",
            executable="simple_drive_controller",
            name="simple_drive_controller",
            output="screen",
        ),
    ])
