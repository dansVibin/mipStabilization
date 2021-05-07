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
#define PI 3.14159265

int main(int argc,char **argv)
{
    double radPerCount = (2*PI)/84;
    double phi = 0;
    double freq = 10;

    std::ofstream outfile ("encoderLog.txt");

    if(rc_encoder_init()){
        fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
        return -1;
    }

    ros::init(argc,argv,"wheel_angle");
    ros::NodeHandle encoder_pub;
    ros::Publisher encoder_publisher = encoder_pub.advertise<std_msgs::Float64>("wheel_angle",1);

    ros::Rate loop_rate(freq);
    while(ros::ok()){
        //external encoder spins opposite to output so multiply by -1
        phi = -1*rc_encoder_read(1)*radPerCount; //convert to radians

        std_msgs::Float64 wheel_angle;
        wheel_angle.data = phi;
        encoder_publisher.publish(wheel_angle);
        loop_rate.sleep();
        }
    rc_encoder_cleanup();
    return 0;
}
