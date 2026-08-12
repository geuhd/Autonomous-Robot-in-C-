#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "practical_nav/map_helpers.hpp"
#include "practical_nav/a_star.hpp"
#include <iostream>

using namespace std;


//create the node subscriber and publisher
rclcpp::Node::SharedPtr node;
rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

//we want to make a variable where we keep the map data
nav_msgs::msg::OccupancyGrid::SharedPtr _map = std::make_shared<nav_msgs::msg::OccupancyGrid>();

//goal is set to false at the moment as we have not reached it yet
bool goalActive = false;

//the extern start and goal in a_star.hpp are actually defined here
cell start;
cell goal;


//costmap is published if there is ab new information in it
//copy published cost map into nwq map we create
void map_handler(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  static bool init_complete = false;
  if (init_complete == false)
  {
    _map->header.frame_id = costmap->header.frame_id;
    _map->info.resolution = costmap->info.resolution;
    _map->info.width = costmap->info.width;
    _map->info.height = costmap->info.height;
    _map->info.origin.position.x = costmap->info.origin.position.x;
    _map->info.origin.position.y = costmap->info.origin.position.y;
    _map->info.origin.orientation.x = costmap->info.origin.orientation.x;
    _map->info.origin.orientation.y = costmap->info.origin.orientation.y;
    _map->info.origin.orientation.z = costmap->info.origin.orientation.z;
    _map->info.origin.orientation.w = costmap->info.origin.orientation.w;
    _map->data.resize(costmap->data.size());
    cout << "Initializing map size " << _map->info.width
         << " x " << _map->info.height << endl;
    init_complete = true;
  }
  //this part is to see the change in the maps
  for (size_t i = 0; i < costmap->data.size(); i++)
    _map->data[i] = costmap->data[i];
}

// the start pose is aalways the lkocation thatttt we are currectly on
bool update_start_cell()
{
  geometry_msgs::msg::TransformStamped t;
  try
  {
    t = tfBuffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    cout << "TF lookup failed: " << ex.what() << endl;
    return false;
  }
  //grid cell is the pose in meters/ map resolution
  start.x = t.transform.translation.x / map_resolution(_map);
  start.y = t.transform.translation.y / map_resolution(_map);

  double qz = t.transform.rotation.z, qw = t.transform.rotation.w;
  double qx = t.transform.rotation.x, qy = t.transform.rotation.y;
  start.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  start.index = getIndex(start.x, start.y, _map);
  return true;
}

// set goal received and set goalActive is true
void set_goal(const geometry_msgs::msg::PoseStamped::SharedPtr desiredPose)
{
  goal.x = (int)(desiredPose->pose.position.x / map_resolution(_map));
  goal.y = (int)(desiredPose->pose.position.y / map_resolution(_map));
  goal.theta = desiredPose->pose.orientation.z;
  goal.index = getIndex(goal.x, goal.y, _map);
  goal.H = 0;
  goalActive = true;
}

void publish_waypoint(cell nextWaypoint)
{
  double res = map_resolution(_map);
  geometry_msgs::msg::PoseStamped wpt;
  wpt.header.frame_id = "map";
  wpt.header.stamp = node->now();
  wpt.pose.position.x = (double)(nextWaypoint.x) * res + res / 2.0;
  wpt.pose.position.y = (double)(nextWaypoint.y) * res + res / 2.0;
  wpt.pose.position.z = 0;
  wpt.pose.orientation.x = 0;
  wpt.pose.orientation.y = 0;
  wpt.pose.orientation.z = nextWaypoint.theta;
  wpt.pose.orientation.w = 0;
  pub->publish(wpt);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    node = rclcpp::Node::make_shared("path_planner");

    tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    //subscribe to map, current pose and goal location
    subMap = node->create_subscription<nav_msgs::msg::OccupancyGrid>("costmap",10, map_handler);
    subGoal = node->create_subscription<geometry_msgs::msg::PoseStamped>("goal_2d", 10, set_goal);
    pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint_2d", 10);

    //check call back every second
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok())
    {
        if (goalActive == true)
        {
            //get the current location from transform data
            //check if there is avalue returned here from the function call
            if (update_start_cell())
            {
                if (start.index == goal.index) goalActive =false;
                else if (find_path() == -1){ cout << "NO PATH FOUND" << endl; goalActive = false; }
            }
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}