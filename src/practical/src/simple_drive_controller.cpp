#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace std;

//Tuning constants 
const double PI = 3.141592;
const double Ka = .35; //ANGULAR VELOCITY GAIN
const double Klv = .65; // LINAR VELOCITY GAIN
const double MAX_LINEAR_VEL =1;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubVel;
geometry_msgs::msg::Twist cmdVel;

nav_msgs::msg::Odometry odom;
geometry_msgs::msg::PoseStamped desired;
bool waypointActive = false;

// callback function that runs when the odometry value arrives on the encoder/odom
void update_pose(const nav_msgs::msg::Odometry & currentOdom)
{
    odom.pose.pose.position.x = currentOdom.pose.pose.position.x;
    odom.pose.pose.position.y = currentOdom.pose.pose.position.y;
    odom.pose.pose.orientation.z = currentOdom.pose.pose.orientation.z;
}

//call back for when waypont data is received from waypoint 2d
void update_goal(const geometry_msgs::msg::PoseStamped & desiredPose){
    desired.pose.position.x = desiredPose.pose.position.x;
    desired.pose.position.y = desiredPose.pose.position.y;
    desired.pose.orientation.z = desiredPose.pose.orientation.z;
    waypointActive = true; // flasg for when we have a goal to drive to
} 

// straight line distance from the position to the goal
double getDistanceError()
{
    double deltaX = desired.pose.position.x -odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y -odom.pose.pose.position.y;

    return sqrt(pow(deltaX,2)+pow(deltaY,2)); 
}

//how musch the robot has to turn to face the goal we have in mond
double getAngularError()
{
    double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
    double thetaBearing = atan2(deltaY,deltaX);
    double angularError = thetaBearing- odom.pose.pose.orientation.z;

    //wrap into -PI  to +PI for shortest way
    angularError =(angularError > PI) ? angularError - (2*PI) : angularError;
    angularError =(angularError < -PI) ? angularError + (2*PI) : angularError;

    return angularError;
}

//the controller: decide cmd_vel and publish it 
void set_velocity()
{
    cmdVel.linear.x = 0;
    cmdVel.angular.z = 0;

    bool angle_met = true;
    bool location_met = true;

    //how far off the Final desired heading is 
    double final_desired_heading_error = desired.pose.orientation.z - odom.pose.pose.orientation.z;

    // are we close enought to the goal 
    if (fabs(getDistanceError()) >= .10){ location_met = false;}
    else if (fabs(getDistanceError()) <0.07){location_met = true;}

    //if not yet at the point, aim at it
    double angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;

    //is our heading "close enough"?
    if (fabs(angularError)> .15){ angle_met = false;}
    else if (fabs(angularError) < .1) { angle_met = true;}

    if (waypointActive == true && angle_met== false)
    {
         // not pointed right yet -> rotate only (proportional to heading error)
        cmdVel.angular.z = Ka * angularError;
        cmdVel.linear.x = 0;
    }
    else if (waypointActive == true && fabs(getDistanceError())>= .1 && location_met ==false)
    {
        // pointed right, not there yet -> drive forward only (proportional to distance)
        cmdVel.linear.x = Klv * getDistanceError();
        cmdVel.angular.z = 0;
    }
    else
    {
        location_met =true;
    }

    if (location_met && fabs(final_desired_heading_error)< .1)
    { 
        waypointActive=false; 
        
    }
    pubVel->publish(cmdVel);
}

int main(int argc, char ** argv ){
    //-1 means "no real gual yet so dont move"
    desired.pose.position.x = -1;

        // start ROS 2
    rclcpp::init(argc,argv);
    node = rclcpp::Node::make_shared("simple_drive_controller");

    // subscribe to the current pose and the oa; 
    auto subPose = node->create_subscription<nav_msgs::msg::Odometry>("encoder/odom",10,update_pose);
    auto subWaypnt = node->create_subscription<geometry_msgs::msg::PoseStamped>("waypoint_2d",10,update_goal);

    // advertise the velocity command output
    pubVel = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    rclcpp::Rate loop_rate(10); //run at 10Hz
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (desired.pose.position.x != -1){
            set_velocity();
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;

}
