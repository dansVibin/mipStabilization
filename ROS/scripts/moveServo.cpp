#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <stdio.h>
#include <stdlib.h>

extern "C"
{
#include <robotcontrol.h>
}

double servo_pos = 0;
int ch = 1;
int freq = 50;

void potPositionCallback(const std_msgs::Int32::ConstPtr & msg){
    servo_pos = double(msg->data)*(.01136) - 1; //servo position as a function of pixel input
    rc_servo_send_pulse_normalized(ch,-1*servo_pos);
    ROS_INFO("%d , %f",msg->data,servo_pos);
}

int main(int argc,char **argv) {
    //read adc
    if (rc_adc_init()){
        fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
        return -1;
    }
    if (rc_adc_batt()<6.0){
    fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive servos\n");
    return -1;
    }

    //initialize PRU
    if(rc_servo_init()) return -1;

    //turn on power
    printf("Turning on 6V Servo Power Rail\n");
    rc_servo_power_rail_en(1);

    //Main loop running at set freq
    ros::init(argc,argv,"pot_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("centroid_NEW",1000,potPositionCallback); //subscribe to centroid topic

    ros::Rate loop_rate(50);
    while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    }

    rc_usleep(50000);
    //turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    rc_dsm_cleanup();
    return 0;
}
