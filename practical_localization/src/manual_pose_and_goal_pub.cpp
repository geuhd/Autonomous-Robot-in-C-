#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>

using namespace std;

// Chapter 11's manual pose/goal publisher. Until Rviz's "2D Pose Estimate"
// and "2D Nav Goal" tools are wired up to something that understands them,
// this lets you set the robot's starting pose and send it goals by typing
// numbers at a terminal.
//
// Same convention as odom_publisher.cpp/simple_drive_controller.cpp/
// path_planner.cpp: orientation.z holds a plain heading in radians, not a
// real quaternion component. That's what those nodes actually read, so
// that's what we publish -- an easier-to-read stand-in for a proper
// quaternion, at the cost of not being usable directly from Rviz's own
// pose/goal tools (those publish real quaternions on different topics).

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("manual_pose_and_goal_pub");

    auto pubInitial = node->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);
    auto pubGoal = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);

    while (rclcpp::ok())
    {
        cout << endl << "Enter (i)nitial pose, (g)oal, or (q)uit: ";
        char cmd;
        cin >> cmd;
        if (cmd == 'q') break;
        if (cmd != 'i' && cmd != 'g')
        {
            cout << "Didn't understand that, try again." << endl;
            continue;
        }

        cout << "x (m): ";
        double xIn; cin >> xIn;
        cout << "y (m): ";
        double yIn; cin >> yIn;
        cout << "theta (rad): ";
        double thetaIn; cin >> thetaIn;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node->now();
        pose.pose.position.x = xIn;
        pose.pose.position.y = yIn;
        pose.pose.position.z = 0;
        pose.pose.orientation.z = thetaIn;   // plain radians, see note above

        if (cmd == 'i')
        {
            pubInitial->publish(pose);
            cout << "Published initial pose." << endl;
        }
        else
        {
            pubGoal->publish(pose);
            cout << "Published goal." << endl;
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
