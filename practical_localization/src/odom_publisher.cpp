#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <iostream>

using namespace std;

// Chapter 11 (wheel odometry) + Chapter 20 (sensor fusion), combined.
//
// NOTE ON HARDWARE: the book's Chapter 20 assumes a BN0055 -- a chip that
// fuses accel+gyro+mag internally and hands you a ready-made orientation
// quaternion, so all its odometry node has to do is read imu.orientation and
// check it isn't flagged "do not use" (orientation_covariance[0] == -1). Our
// imu_publisher.cpp is a bare MPU6050 + QMC5883L instead, and it deliberately
// sets orientation_covariance[0] = -1 because it has no fused orientation to
// give -- just raw linear_acceleration/angular_velocity plus a separate
// magnetic_field message. So instead of reading imu.orientation, we fuse it
// ourselves with a simple complementary filter: integrate the gyro's z-axis
// for good short-term heading, pulled back toward the compass heading from
// the magnetometer so it doesn't drift forever. Same idea as the book's
// sensor fusion chapter, just done in software because our IMU doesn't do it
// for us in hardware. Until the IMU/compass have sent at least one message
// each, we fall back to plain Chapter-11 wheel-differential heading.

const double WHEEL_BASE = 0.35;    // metres between the two drive wheels -- MEASURE YOURS AND UPDATE THIS
const double TICKS_PER_M = 2250;   // must match simple_diff.cpp
const double ALPHA = 0.98;         // complementary filter weight: trust the gyro this much, the compass the rest

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

// current pose estimate -- x/y in metres, theta in radians
double x = 0, y = 0, theta = 0;
bool poseInitialized = false;   // don't integrate/publish until initial_2d arrives, like the book

// raw sensor state, updated by the callbacks below and consumed by the main loop
int leftCount = 0, rightCount = 0;
int prevLeftCount = 0, prevRightCount = 0;
bool firstTickPair = true;   // so we don't take a huge delta from count 0 on the first pass
double gyroZ = 0;            // rad/s, from imu_data
double magX = 0, magY = 0;   // tesla, from magnetic_field
bool haveImu = false, haveMag = false;

// seed the starting pose from manual_pose_and_goal_pub (topic: initial_2d).
// same "easy to read" convention as the rest of this codebase: orientation.z
// is a plain heading in radians, not a real quaternion component.
void set_initial_pose(const geometry_msgs::msg::PoseStamped & initialPose)
{
    x = initialPose.pose.position.x;
    y = initialPose.pose.position.y;
    theta = initialPose.pose.orientation.z;
    prevLeftCount = leftCount;
    prevRightCount = rightCount;
    firstTickPair = true;
    poseInitialized = true;
    cout << "odom_publisher: initial pose set to x=" << x << " y=" << y << " theta=" << theta << endl;
}

void left_tick_handler(const std_msgs::msg::Int16 & lCount) { leftCount = lCount.data; }
void right_tick_handler(const std_msgs::msg::Int16 & rCount) { rightCount = rCount.data; }

void imu_handler(const sensor_msgs::msg::Imu & imu)
{
    gyroZ = imu.angular_velocity.z;
    haveImu = true;
}

// compass heading from the magnetometer -- flat-mount assumption, no tilt
// compensation (the book keeps it simple too; add tilt compensation later
// with the accelerometer if your IMU isn't mounted level)
void mag_handler(const sensor_msgs::msg::MagneticField & mag)
{
    magX = mag.magnetic_field.x;
    magY = mag.magnetic_field.y;
    haveMag = true;
}

// fold a raw tick count into a signed delta since last time, same unwrap
// logic tick_publisher.cpp/simple_diff.cpp use for the int16 rollover
int ticks_since(int newCount, int & lastCount)
{
    int cycleDistance = (65535 + newCount - lastCount) % 65535;
    if (cycleDistance > 10000)
        cycleDistance = 0 - (65535 - cycleDistance);
    lastCount = newCount;
    return cycleDistance;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("odom_publisher");

    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto subInitial = node->create_subscription<geometry_msgs::msg::PoseStamped>("initial_2d", 10, set_initial_pose);
    auto subImu = node->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, imu_handler);
    auto subMag = node->create_subscription<sensor_msgs::msg::MagneticField>("magnetic_field", 10, mag_handler);
    auto subLeft = node->create_subscription<std_msgs::msg::Int16>("leftWheel", 10, left_tick_handler);
    auto subRight = node->create_subscription<std_msgs::msg::Int16>("rightWheel", 10, right_tick_handler);

    pubOdom = node->create_publisher<nav_msgs::msg::Odometry>("encoder/odom", 10);

    cout << "odom_publisher: waiting for an initial pose on initial_2d before integrating..." << endl;

    double lastLoopTime = 0;
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (poseInitialized)
        {
            double deltaLeft = firstTickPair ? 0 : ticks_since(leftCount, prevLeftCount) / TICKS_PER_M;
            double deltaRight = firstTickPair ? 0 : ticks_since(rightCount, prevRightCount) / TICKS_PER_M;
            if (firstTickPair) { prevLeftCount = leftCount; prevRightCount = rightCount; firstTickPair = false; }
            double deltaDistance = (deltaLeft + deltaRight) / 2.0;

            double now = node->now().seconds();
            double dt = (lastLoopTime > 0) ? now - lastLoopTime : 0;
            lastLoopTime = now;

            if (haveImu && haveMag)
            {
                // Chapter 20: gyro-integrated heading, corrected toward the compass
                double compassHeading = atan2(magY, magX);
                double gyroHeading = theta + gyroZ * dt;
                theta = ALPHA * gyroHeading + (1.0 - ALPHA) * compassHeading;
            }
            else
            {
                // Chapter 11 fallback: plain wheel-differential dead reckoning
                // until the IMU/compass come online
                theta += (deltaRight - deltaLeft) / WHEEL_BASE;
            }

            x += deltaDistance * cos(theta);
            y += deltaDistance * sin(theta);

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = node->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0;
            // matches simple_drive_controller.cpp / path_planner.cpp, which both
            // read orientation.z straight back out as a plain radian heading --
            // NOT a real quaternion component. Left as-is on purpose.
            odom.pose.pose.orientation.z = theta;
            pubOdom->publish(odom);

            // the tf DOES need a real quaternion, since path_planner.cpp looks
            // this transform up through tf2_ros and does proper quaternion math
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = odom.header.stamp;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0;
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tfBroadcaster->sendTransform(t);
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
