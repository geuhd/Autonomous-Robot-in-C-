#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <fcntl.h>
#include <termios.h> //configure the uart for the system
#include <unistd.h>
#include <cstdint>
#include <iostream>

using namespace std;

const char *SERIAL_PORT ="/dev/ttyAMA0";
sensor_msgs::msg::Range range;

//configure the serial port for the device
int open_serial(const char *port)
{
    //open(...): Opens the serial device. O_RDWR grants read and write access. 
    //O_NOCTTY ensures that this serial port does not accidentally become the controlling terminal for your OS process.
    int fd =open(port,O_RDWR | O_NOCTTY);
    //if opeing the serial device fails it should return -1
    if (fd<0) return -1;

    //create a structure that holds the config data 
    struct termios tty;
    //tcgetattr: Loads the current configuration of the serial port into the tty struct.
    tcgetattr(fd,&tty);
    //cfsetispeed / cfsetospeed: Sets both the input and output baud rate to 115200 to match the TF-Luna's factory default.
    cfsetispeed(&tty,B115200);
    cfsetospeed(&tty,B115200);

    //FORCE THE DATA SIZE TO BE 8 BITS
    tty.c_cflag =(tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL|CREAD); //CLOCAL | CREAD: Ignores modem control lines (making it a direct local connection) and enables the receiver to read data.
    tty.c_cflag &= ~PARENB; //no parity
    tty.c_cflag &= ~CSTOPB; // enforce one stop bit                    
    tty.c_cflag &= ~CRTSCTS; //disables flow control. TF LUNA JUST SENDS THE DATA WITHOUT WAITING FOR SIGNAL TO SEND
    tty.c_lflag =0;
    tty.c_oflag =0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

float get_range(int fd)
{
    tcflush(fd, TCIFLUSH);
    //buf[9]: Creates a 9-byte array to store one complete TF-Luna data frame.
    unsigned char buf[9];
    while(rclcpp::ok())
    {
        unsigned char b =0;
        //hunt for the header one byte at a time so a stray 0x59 in the data always resyncs us
        if (read(fd, &b,1)!= 1 || b != 0x59) continue;
        if (read(fd, &b, 1) != 1 || b != 0x59) continue;

        //The sensor starts every frame with two 0x59 bytes. The code reads one byte at a time.
        //If it doesn't read exactly one byte, or if the byte isn't 0x59, it skips the rest of the loop (continue) and tries again.
        //Once two 0x59 bytes are found in a row, it logs them into the buffer.
        buf[0]= 0x59;
        buf[1]= 0x59;

        //collect the 7 data bytes 
        int got =0;
        while(got<7){
            int n = read(fd,buf+2+got,7-got);
            if (n>0) got +=n;
        }
        //add first 8 bytes
        //checks if it matches the 9th byte
        //if it doesnt it is corrupted -> resync from the next 0x59
        int sum = 0;
        for (int i =0; i<8; i++) sum+=buf[i];
        if ((sum & 0xFF) != buf[8]) continue;

        //distance calculation.
        //value broken into 2 parts, value in cm
        int dist_cm = buf[2] + (buf[3] << 8);
        //signal strength: how much light came back. low = untrustworthy, 65535 = saturated
        int strength = buf[4] + (buf[5] << 8);
	RCLCPP_INFO(rclcpp::get_logger("tf_luna"), "dist=%d cm  strength=%d", dist_cm, strength);   // <-- add this line only

        //Benewake: ignore readings with weak or saturated signal, they are not real targets
        if (strength < 100 || strength == 65535) return -1.0f;

        return dist_cm / 100.0f;
    }
    return -1;
}
//main function and in here we would start the ros node
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    //make the ros node and make a publsher for the range
    auto node = rclcpp::Node::make_shared("tf_luna_publisher");
    auto pub = node->create_publisher<sensor_msgs::msg::Range>("range",10);

    int fd = open_serial(SERIAL_PORT);
    if (fd>= 0)cout << "TF-Luna serial opened ok on " << SERIAL_PORT << endl;
    else {cout << "Failed to open " << SERIAL_PORT << " check wiring/permissions" << endl;return -1;}

    //hardware initalisation
    range.header.frame_id = "tf_luna";
    range.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range.field_of_view = 0.0349;
    range.min_range = 0.2;
    range.max_range = 8.0;

    //ros 2 looprclcpp::Rate loop_rate(10);
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        range.header.stamp = node->now();
        range.range = get_range(fd);
        pub->publish(range);
        loop_rate.sleep();
    }
    close(fd);
    rclcpp::shutdown();
    return 0;
}
