#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <rgpio.h>
#include <cstdint>
#include <iostream>

using namespace std;

//declaration of constants aand globals
const int leftEncoder = 22;
const int rightEncoder =23;
const int leftReverse =13;
const int rightReverse = 19;

const int GPIO_CHIP = 0;

const int encoderMin = -32768;
const int encoderMax = 32768;

std_msgs::msg::Int16 leftCount;      
std_msgs::msg::Int16 rightCount;

int sbc = -1;   
int h = -1;  

//THE CALL BACK DOES NOT KOW IF THE WHEEL IS REVERSING SO We need to check the pin that is on
void left_event(int,int,int,int, uint64_t ,void *)
{
    if(gpio_read(sbc,h,leftReverse) ==0)
    {
        if(leftCount.data == encoderMin) leftCount.data = encoderMax; //rollunder
        else leftCount.data --;

    }
    else 
    {
        if (leftCount.data == encoderMax) leftCount.data = encoderMin; //rollover
        else leftCount.data++;
    }
}

void right_event(int, int, int, int, uint64_t, void *)
{
  if (gpio_read(sbc, h, rightReverse) == 0)
  {
    if (rightCount.data == encoderMin) rightCount.data = encoderMax;
    else rightCount.data--;
  }
  else
  {
    if (rightCount.data == encoderMax) rightCount.data = encoderMin;
    else rightCount.data++;
  }
}

int GpioSetup()
{
    sbc = rgpiod_start(NULL,NULL);
    if (sbc<0) return sbc;
    h = gpiochip_open(sbc,GPIO_CHIP);
    if (h<0) return h;

    //claim pins as inputs with pullup
    gpio_claim_input(sbc,h,LG_SET_PULL_UP,leftReverse);
    gpio_claim_input(sbc, h, LG_SET_PULL_UP, rightReverse);

    //encoder pins are alert sources on both ends
    gpio_claim_alert(sbc, h, LG_SET_PULL_UP, BOTH_EDGES, leftEncoder, -1);
    gpio_claim_alert(sbc, h, LG_SET_PULL_UP, BOTH_EDGES, rightEncoder, -1);
    return sbc;
}

//main function is a bit more basic 
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("encoder_ticks");

    //publish the right and the left encoder ticks
    auto pubLeft = node->create_publisher<std_msgs::msg::Int16>("leftWheel",10);
    auto pubRight = node->create_publisher<std_msgs::msg::Int16>("rightWheel", 10);

    if (GpioSetup() >= 0 && h>=0) {cout << "rgpiod interface started ok, sbc = " << sbc << endl;}
    else { cout << "Failed to connect to rgpiod Daemon - is it running? (sudo rgpiod)" << endl; return -1; }

    //register the edge callbacks
    int cbLeft = callback(sbc,h,leftEncoder,BOTH_EDGES,left_event,NULL);
    int cbRight = callback(sbc,h,rightEncoder,BOTH_EDGES,right_event,NULL);

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok())
    {
        pubLeft->publish(leftCount);                             // ROS1: pubLeft.publish(leftCount)
        pubRight->publish(rightCount);
        rclcpp::spin_some(node);                                 // ROS1: ros::spinOnce()
        loop_rate.sleep();
    }
    //terminate callback and the rgpio
    callback_cancel(cbLeft);                                    
    callback_cancel(cbRight);
    gpiochip_close(sbc, h);                                     
    rgpiod_stop(sbc);                                          
    rclcpp::shutdown();                                        
    return 0;

}