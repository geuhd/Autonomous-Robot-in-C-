#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cstdlib>
#include <iostream>
using namespace std;

// declaring variables 
geometry_msgs::Twist cmdVel;
geometry_msgs::Pose2D current;
geometry_msgs::Pose2D desired;

const double GOAL 1.5;

//this is the gain of the speed
const double K1 =1.0;
const double distanceTolerance = 0.1;

void msic_setup()
{
    desired.x = 5.54;
    desired.y = 5.54;
    cmdVel.linear.x =0;
    cmdVel.linear.y =0;
    cmdVel.linear.z =0;
    cmdVel.angular.x =0;
    cmdVel.angular.y =0;
    cmdVel.angular.z =0;
}

//Callback to update the current pose
void update_pose(const turtlesim::PoseConstPtr &currentPose)
{
    current.x = currentPose->x;
    current.y = currentPose->y;
    current.theta = currentPose->theta;
}

//---------------------------Hepler Functions-----------//
double getDistanceError()
{
    return desired.x - current.x;
}

//--- Controller functions 
void set_velocity()
{
    if(abs(getDistanceError())> distanceTolerance)
    {
        cmdVel.linear.x = K1 * getDistanceError();
    }
    else
    {
        cout<<"I have arrived"<<endl;
        cmdVel.linear.x =0;
    }
}

int main(int argc, char **argv)
{
    // initialise variables 
    msic_setup();
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("go_to_x");
//from the node class we want the create subscription method and they type we want is turtle::msg::Pose> with data buffer 10
    auto subCurrentPose = node->create_subscription<turtle::msg::Pose>("turtle1/pose",10,update_pose)

    //node object with the create a publisher that publishes a message to the turtle velocity
    auto pubVelocity = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10)

    /
    rclcpp::Rate loop_rate(10);

    //while rclcpp is okay thus the connection is still there
    while(rclcpp::ok())
    {
        
        rclcpp::spin_some(node);
        set_velocity();
        pubVelocity->publish(cmdVel);

        cout << "goal x = " << desired.x << endl
            << "current x = " << current.x << endl
            << " disError = " << getDistanceError() << endl
            << "cmd_vel = " << cmdVel.linear.x << endl;

        loop_rate.sleep();


    }
    rclcpp::shutdon();
    return 0; 
}

