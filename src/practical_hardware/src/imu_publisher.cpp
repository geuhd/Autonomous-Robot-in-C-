#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <rgpio.h> // lg library -> its I2C functions
#include <cstdint> //int16_t / uint8_t
#include <cmath>
#include <iostream>

using namespace std;

//global constants and addresses for the sensors
const int I2C_BUS =1; // Raspberry Pi's user I2C bus = /dev/i2c-1
const int MPU_ADDR = 0x68;// MPU6050's fixed I2C address
const int QMC_ADDR =0x0D; // QMC5883L's fixed I2C address

//needed registers for the sensor 
const int MPU_PWR_MGMT_1 = 0x6B;   // power control: write 0 here to wake the chip
const int MPU_ACCEL_XOUT = 0x3B;   // first of 6 accel bytes
const int MPU_GYRO_XOUT  = 0x43;   // first of 6 gyro bytes
const int QMC_DATA       = 0x00;   // first of 6 mag bytes
const int QMC_CONTROL1   = 0x09;   // QMC mode/rate/range config
const int QMC_SET_RESET  = 0x0B;   // QMC set/reset period

//scales needed to change the real data to actual values we can use
const double ACCEL_SCALE = 9.80665 / 16384.0;      // MPU6050 gives 16384 counts per g -> m/s^2
const double GYRO_SCALE  = (M_PI / 180.0) / 131.0; // 131 counts per deg/s -> rad/s
const double MAG_SCALE   = 1e-4 / 3000.0;          // 3000 counts per Gauss, 1 Gauss = 1e-4 T -> Tesla

//rgpio connection plus the I2C  handler for the 2 sensors
int sbc =-1;
int mpu =-1;
int qmc =-1;

int16_t to_int16(uint8_t hi, uint8_t lo)
{
  return (int16_t)((hi << 8) | lo);   // put hi in the top 8 bits, lo in the bottom 8
}

//I2C setup and RGIO start
int I2cSetup()
{
    sbc = rgpiod_start(NULL, NULL); 
    if (sbc <0) return sbc;
    mpu =i2c_open(sbc,I2C_BUS,MPU_ADDR,0); //open the MPU6050 on I2C bus 1
    qmc = i2c_open(sbc,I2C_BUS,QMC_ADDR,0); //open the QMC5883L QMC5883L on I2C bus 1
    if (mpu< 0||qmc<0) return -1;

    //for the MPU6050 write 0 to power manageemnt to wake it
    i2c_write_byte_data(sbc,mpu,MPU_PWR_MGMT_1,0x00);
    // QMC5883L config: set/reset period (datasheet requires 0x01)
    i2c_write_byte_data(sbc, qmc, QMC_SET_RESET, 0x01);
    // control reg 0x09 = 0x1D -> continuous, 200 Hz, +/-8 Gauss, OSR 512
    i2c_write_byte_data(sbc, qmc, QMC_CONTROL1, 0x1D);

    return sbc;
}

//main function that starts the ros 2 node
int main(int argc, char**argv)
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("imu_publisher");

    auto imuPub = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    auto magPub = node->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 10);

    if(I2cSetup()>=0 && mpu>=0 && qmc>=0){cout << "I2C sensors opened ok (MPU6050 + QMC5883L)" << endl;}
    else {cout << "Failed to open I2C sensors - is rgpiod running and I2C enabled?" << endl;return -1;}

    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::MagneticField mag;

    imu.header.frame_id ="imu_link";
    imu.orientation_covariance[0] =-1; // no fused orientation available thus we turn it off as suggested
    mag.header.frame_id = "imu_link";

    unsigned char buf[6];   // scratch buffer for the 6-byte reads

    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok())
    {
        rclcpp::Time now = node->now();

        // MPU6050 accel (6 bytes from 0x3B, BIG-endian)
        i2c_read_i2c_block_data(sbc, mpu, MPU_ACCEL_XOUT, (char *)buf, 6);
        int16_t ax = to_int16(buf[0], buf[1]);
        int16_t ay = to_int16(buf[2], buf[3]);
        int16_t az = to_int16(buf[4], buf[5]);

        // MPU6050 gyro (6 bytes from 0x43, BIG-endian)
        i2c_read_i2c_block_data(sbc, mpu, MPU_GYRO_XOUT, (char *)buf, 6);
        int16_t gx = to_int16(buf[0], buf[1]);
        int16_t gy = to_int16(buf[2], buf[3]);
        int16_t gz = to_int16(buf[4], buf[5]);

        // QMC5883L mag (6 bytes from 0x00, LITTLE-endian -> swap args)
        i2c_read_i2c_block_data(sbc, qmc, QMC_DATA, (char *)buf, 6);
        int16_t mx = to_int16(buf[1], buf[0]);
        int16_t my = to_int16(buf[3], buf[2]);
        int16_t mz = to_int16(buf[5], buf[4]);

        //now we have the data we would need to fill the IMU message amdf the mag message with the data
        imu.header.stamp =now;
        imu.linear_acceleration.x = ax*ACCEL_SCALE;
        imu.linear_acceleration.y =ay*ACCEL_SCALE;
        imu.linear_acceleration.z = az * ACCEL_SCALE;
        imu.angular_velocity.x = gx * GYRO_SCALE;
        imu.angular_velocity.y = gy * GYRO_SCALE;
        imu.angular_velocity.z = gz * GYRO_SCALE;
        imuPub->publish(imu);

        mag.header.stamp = now;
        mag.magnetic_field.x = mx * MAG_SCALE;
        mag.magnetic_field.y = my * MAG_SCALE;
        mag.magnetic_field.z = mz * MAG_SCALE;
        magPub->publish(mag);

        loop_rate.sleep();
    }

    //close the i2c annd stop rgpiod
    i2c_close(sbc,mpu);
    i2c_close(sbc,qmc);
    rgpiod_stop(sbc);
    return 0;

}
