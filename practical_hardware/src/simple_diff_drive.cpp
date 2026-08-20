#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <iostream>
#include <rgpio.h>

using namespace std;

//Declaration of constants needed in the project

const int PWM_INCREMENT = 2;
const double TICKS_PER_M = 2250;
const int MIN_PWM = 55;200
const int MAX_PWM =255;
const float PWM_FREQ = 1000; //THIS IS NEEDED AS THE NEW LIBRARY USES A STATED VALUE INSTEAD OF THE OD ONE THAT WOUDL USE A DEFAULT VALUE

//left and right motors and their direction pin assignment
const int PWM_L = 21;
const int MOTOR_L_FWD =26;
const int MOTOR_L_REV = 13;
const int PWM_R = 12;
const int MOTOR_R_FWD =20;
const int MOTOR_R_REV =19;

//gpiochip0 on the Pi 3B+
const int GPIO_CHIP = 0;

//INITALISE THE VELOCITIES OF THE RIGHT AND LEFT MOTORS 
double leftVelocity =0;
double rightVelocity =0;
double leftPWMReq =0;
double rightPWMReq = 0;
double lastCmdMsgRcvd =0;

//the direction each wheel is being driven, sent to tick_publisher
bool leftReversing = false;
bool rightReversing = false;200


// rgpio needs TWO handles: sbc = daemon connection, h = gpiochip
int sbc = -1;
int h = -1;


// A node is create an empty node called Node:In ROS 1, getting the time was a free-standing call — ros::Time::now().toSec() — that didn't need anything. 
//In ROS 2, now() is a method on the node object (node->now()). 
//So any function that reads the clock must be able to reach the node.

rclcpp::Node::SharedPtr node;

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubLeftDir;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubRightDir;

//calculate the velocity of the left wheel
void Calc_Left_Vel(const std_msgs::msg::Int16 & lCount)
{
    static double lastTime =0;
    static int lastCount =0;

    //THis handles the encoder wrapping arounf
    int cycleDistance = (65535 + lCount.data - lastCount) % 65535;

    if (cycleDistance >10000)
        cycleDistance = 0 - (65535 -cycleDistance);

    double dt = node->now().seconds() - lastTime;
    if(dt >1e-6) leftVelocity = cycleDistance/TICKS_PER_M/dt;
    lastCount =lCount.data;
    lastTime= node->now().seconds();

}
//calculate the velocity of the right wheel 
void Calc_Right_Vel(const std_msgs::msg::Int16 & rCount)   

{
    static double lastTime =0;
    static int lastCount =0;

    //THis handles the encoder wrapping arounf
    int cycleDistance = (65535 + rCount.data - lastCount) % 65535;

    if (cycleDistance >10000)
        cycleDistance = 0 - (65535 -cycleDistance);

    double dt = node->now().seconds() - lastTime;
    if(dt >1e-6) rightVelocity = cycleDistance/TICKS_PER_M/dt;
    lastCount =rCount.data;
    lastTime= node->now().seconds();

}

//function to set the PWM speeds
void Set_Speeds(const geometry_msgs::msg::Twist & cmdVel)
{
    lastCmdMsgRcvd = node->now().seconds();

    if(cmdVel.angular.z>.10){leftPWMReq = -70; rightPWMReq =70;} // turn left
    else if (cmdVel.angular.z < -.10){leftPWMReq = 70; rightPWMReq = -70;} // turn right

    else if (fabs(cmdVel.linear.x)>0.01)
    {
        leftPWMReq = 400 * cmdVel.linear.x + 60;
        rightPWMReq = 400 * cmdVel.linear.x + 60;

        //average difference in the wheel speed of the last 3 cycles
        double angularVelDiff = leftVelocity - rightVelocity;
        static double prevDiff =0;
        static double prevDiff2 = 0;

        double avgAngularDiff = (prevDiff + prevDiff2 + angularVelDiff)/3;
        prevDiff2 = prevDiff;
        prevDiff = angularVelDiff;

        //here we  are applying correction so the wheels can continue to go straight
        //leftPWMReq -= (int)(avgAngularDiff*125);
        //rightPWMReq += (int)(avgAngularDiff*125);
    }
    else {leftPWMReq = 0; rightPWMReq = 0;}

    // for the values that do not cause the robot to move we should set these to 0
    leftPWMReq = (fabs(leftPWMReq)< MIN_PWM) ? 0 : leftPWMReq;
    rightPWMReq =(fabs(rightPWMReq)< MIN_PWM) ? 0 : rightPWMReq;
}

// This functions set the values we calcuated in the previous function to the GPIO pins
//It is similar to the chapters of the previous book
void set_pin_values()
{
    static int leftPwmOut =0;
    static int rightPwmOut =0 ;

    //rgpio  uses gpio_write(sbc,h,PIN,val) 
    // the h is the handle  
    if(leftPWMReq >0) {gpio_write(sbc,h,MOTOR_L_REV,1); gpio_write(sbc,h,MOTOR_L_FWD,0); leftReversing = false;}   // LEFT forwad
    else if (leftPWMReq<0){gpio_write(sbc,h,MOTOR_L_FWD,1); gpio_write(sbc,h,MOTOR_L_REV,0); leftReversing = true;}   //LEFT MOTOR REVESE
    else if (leftPWMReq ==0 && leftPwmOut ==0) {gpio_write(sbc,h,MOTOR_L_FWD,1); gpio_write(sbc,h,MOTOR_L_REV,1);}//LEFT 
    
    //sending our PWM signal
    if (fabs(leftPWMReq)>leftPwmOut){ leftPwmOut+= PWM_INCREMENT;}
    else if (fabs(leftPWMReq)<leftPwmOut){ leftPwmOut-= PWM_INCREMENT;}

    //check if the PWM signal is out of bounds or under the limit
    leftPwmOut = (leftPwmOut>MAX_PWM) ? MAX_PWM : leftPwmOut;
    leftPwmOut = (leftPwmOut<0)  ? 0:leftPwmOut;

    //for rgpio we should use tx_pwm // tx_pwm(sbc,h,PWM_L,PWM_FREQ,XXXX)
    tx_pwm(sbc,h,PWM_L,PWM_FREQ,leftPwmOut/255.0f*100.0f,0,0);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //same steps but for the right motor 
    if(rightPWMReq >0) {gpio_write(sbc,h,MOTOR_R_REV,1); gpio_write(sbc,h,MOTOR_R_FWD,0); rightReversing = false;}   // LEFT forwad
    else if (rightPWMReq<0){gpio_write(sbc,h,MOTOR_R_FWD,1); gpio_write(sbc,h,MOTOR_R_REV,0); rightReversing = true;}   //LEFT MOTOR REVESE
    else if (rightPWMReq ==0 && rightPwmOut ==0) {gpio_write(sbc,h,MOTOR_R_FWD,1); gpio_write(sbc,h,MOTOR_R_REV,1);}//LEFT 
    
    //sending our PWM signal
    if (fabs(rightPWMReq)>rightPwmOut){ rightPwmOut+= PWM_INCREMENT;}
    else if (fabs(rightPWMReq)<rightPwmOut){ rightPwmOut-= PWM_INCREMENT;}

    //check if the PWM signal is out of bounds or under the limit
    rightPwmOut = (rightPwmOut>MAX_PWM) ? MAX_PWM : rightPwmOut;
    rightPwmOut = (rightPwmOut<0)  ? 0:rightPwmOut;

    //for rgpio we should use tx_pwm // tx_pwm(sbc,h,PWM_L,PWM_FREQ,XXXX)
    tx_pwm(sbc,h,PWM_R,PWM_FREQ,rightPwmOut/255.0f*100.0f,0,0);

    //tell tick_publisher which way we are driving instead of it reading our pins
    std_msgs::msg::Bool leftDir; leftDir.data = leftReversing;
    std_msgs::msg::Bool rightDir; rightDir.data = rightReversing;
    pubLeftDir->publish(leftDir);
    pubRightDir->publish(rightDir);

}
// This section is dedicated to the GPIO set up using rgpio
int GpioSetup()
{
    sbc = rgpiod_start(NULL,NULL);
    if(sbc<0)return sbc;
    h = gpiochip_open(sbc,GPIO_CHIP);
    if(h<0) return h;

    //set the pin modes for the gpio pins
    //rgpio:: gpio_clain_output(sbc,h,flags,PIN, initalValue)
    //a line already claimed by another process returns an error here, so check it
    if(gpio_claim_output(sbc,h,0,PWM_L,0)<0){cout<<"Failed to claim GPIO "<<PWM_L<<endl; return -1;}
    if(gpio_claim_output(sbc,h,0,MOTOR_L_FWD,1)<0){cout<<"Failed to claim GPIO "<<MOTOR_L_FWD<<endl; return -1;}
    if(gpio_claim_output(sbc,h,0,MOTOR_L_REV,1)<0){cout<<"Failed to claim GPIO "<<MOTOR_L_REV<<endl; return -1;}
    if(gpio_claim_output(sbc,h,0,PWM_R,0)<0){cout<<"Failed to claim GPIO "<<PWM_R<<endl; return -1;}
    if(gpio_claim_output(sbc,h,0,MOTOR_R_FWD,1)<0){cout<<"Failed to claim GPIO "<<MOTOR_R_FWD<<endl; return -1;}
    if(gpio_claim_output(sbc,h,0,MOTOR_R_REV,1)<0){cout<<"Failed to claim GPIO "<<MOTOR_R_REV<<endl; return -1;}

    //make sure both channels are off before anything else happens
    tx_pwm(sbc,h,PWM_L,PWM_FREQ,0,0,0);
    tx_pwm(sbc,h,PWM_R,PWM_FREQ,0,0,0);

    return sbc;

}

//main function of the node 
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    node = rclcpp::Node::make_shared("simple_diff_drive");

    sbc = GpioSetup();
    if(sbc>=0 && h>=0){ cout<<"rgpio interface started ok, sbc = "<<sbc<<endl;}
    else { cout<<"Failed to connect to rgpiod Daemon - is it running? (sudo rgpiod)"<<endl; return -1;}
    

    auto subRCounts = node->create_subscription<std_msgs::msg::Int16>("rightWheel",10,Calc_Right_Vel);
    auto subLCounts = node->create_subscription<std_msgs::msg::Int16>("leftWheel",10,Calc_Left_Vel);
    auto subVelocity = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,Set_Speeds);

    pubLeftDir = node->create_publisher<std_msgs::msg::Bool>("left_reversing",10);
    pubRightDir = node->create_publisher<std_msgs::msg::Bool>("right_reversing",10);

    rclcpp::Rate loop_rate(50);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if(node->now().seconds()-lastCmdMsgRcvd>1)
        {
            cout<<"Not Receiving CMD Vel-Stopping motors"<<endl;
            leftPWMReq=0;
            rightPWMReq=0;
        }
        set_pin_values();
        loop_rate.sleep();
    }
    tx_pwm(sbc,h,PWM_L,PWM_FREQ,0,0,0);
    tx_pwm(sbc,h,PWM_R,PWM_FREQ,0,0,0);

    gpio_write(sbc,h,MOTOR_L_FWD,1);
    gpio_write(sbc,h,MOTOR_L_REV,1);
    gpio_write(sbc,h,MOTOR_R_FWD,1);
    gpio_write(sbc,h,MOTOR_R_REV,1);

    gpiochip_close(sbc,h);
    rgpiod_stop(sbc);
    rclcpp::shutdown();
    return 0;

}