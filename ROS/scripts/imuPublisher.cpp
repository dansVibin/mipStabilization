#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <fstream>

extern "C"
{
#include <robotcontrol.h>
}

#define DEG_TO_RAD   0.0174532925199
#define PI 3.14159265

int main(int argc,char **argv)
{
    rc_mpu_data_t dataIMU; //struct to hold new data
    rc_mpu_config_t conf = rc_mpu_default_config();

    double theta [3]= {0.0,0.0,0.0}; //{a,g,f}
    double ratio = 0.98;
    double freq = 100;
    double dt = 1/freq;

    if(rc_mpu_initialize(&dataIMU, conf)){
        fprintf(stderr,"rc_mpu_initialize_failed\n");
        return -1;
    }

    ros::init(argc,argv,"body_angle");
    ros::NodeHandle imu_pub;
    ros::Publisher imu_publisher = imu_pub.advertise<std_msgs::Float64>("body_angle",1);

    ros::Rate loop_rate(freq);
    while(ros::ok()){
        //catch mpu reading error
        if(rc_mpu_read_accel(&dataIMU)<0){
            printf("read accel data failed\n");
        }
        if(rc_mpu_read_gyro(&dataIMU)<0){
            printf("read gyro data failed\n");
        }
        //take 2% of accel angle found using arctan of (-z/y)
        theta[0] = (1-ratio) * atan2(-dataIMU.accel[2],dataIMU.accel[1]);

        //take 98% of gyro angle found using euler integration
        theta[1] = ratio * (theta[2] + dt*dataIMU.gyro[0]*DEG_TO_RAD);

        //final angle defined as a composite of accel angle and gryo angle
        theta[2] = (theta[0] + theta[1]);

        std_msgs::Float64 body_angle;
        body_angle.data = theta[2];
        imu_publisher.publish(body_angle);

        loop_rate.sleep(); //sleep defined by freq
        }
    rc_mpu_power_off();
    return 0;
}
