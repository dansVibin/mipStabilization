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

bool startInit = false;
bool startPrgm = false;
int bttnCount = 0;

static void __on_mode_press(void)
{
   return;
}
static void __on_mode_release(void)
{
   bttnCount += 1;
   if (bttnCount >= 1) startInit = true;
   if (bttnCount == 2) startPrgm = true;
   return;
}
//Dz2 controller = (b1*z + b0)/(z + a0)
double gain = 1;
//10Hz
double Dz2num [5] = {0.357*gain , -0.3249*gain, -0.3438*gain, 0.171*gain, 0.1418*gain}; //{b0,b1,b2,b3,b4}
double Dz2den [4] = {0.1776,  0.009047, -0.8686, -0.318};//{a0,a1,a2,a3}

double thetaRef [5]= {0,0,0,0,0}; //{k,k-1,k-2}
double phiRef = 0;
double phiMeasured  = 0;
double phiError [5]= {0,0,0,0,0}; //{k,k-1,k-2}

void wheelAngleCallback(const std_msgs::Float64::ConstPtr & msg){
   if (startInit == true) phiMeasured = msg -> data;
}

int main(int argc,char ** argv)
{
    double sampleFreq = 10;
    double dt = 1/sampleFreq;
    double timeCount = 0;
    std::ofstream outfile ("outerLoopLog.txt");

    //initialize beaglebone buttons
    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
                                    RC_BTN_DEBOUNCE_DEFAULT_US)){
       fprintf(stderr,"ERROR: failed to init buttons\n");
       return -1;
    }
    rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, __on_mode_release);

    //set ros node
    ros::init(argc,argv,"outerloop");
    ros::NodeHandle outer;

    //subscribe to /wheel_angle node and publish to /theta_reference topic
    ros::Subscriber phi_sub = outer.subscribe("wheel_angle",1,wheelAngleCallback);
    ros::Publisher ref_pub = outer.advertise<std_msgs::Float64>("theta_reference",1);

    //outfile << Dz2num[0] << " " << Dz2num[1] << " " << Dz2num[2] << " " << Dz2den[0] << " " << Dz2den[1] << std::endl;

    ros::Rate loop_rate(sampleFreq);
    while(ros::ok()){
        phiError[0] = phiRef - phiMeasured;
  
  //difference equation
  thetaRef[0] = Dz2num[4]*phiError[0] +Dz2num[3]*phiError[1] +Dz2num[2]*phiError[2] +Dz2num[1]*phiError[3] +Dz2num[0]*phiError[4]      -Dz2den[3]*thetaRef[1] -Dz2den[2]*thetaRef[2] -Dz2den[1]*thetaRef[3] -Dz2den[0]*thetaRef[4];

        std_msgs::Float64 theta_reference;
        theta_reference.data = thetaRef[0];
        ref_pub.publish(theta_reference);


        //if (startPrgm == true){
        //outfile << timeCount << " " << phiRef << " " << phiMeasured << " " << phiError[0] << " " << thetaRef[0] << std::endl;
        //timeCount += dt;
        //}

        phiError[4] = phiError[3];
        phiError[3] = phiError[2];
        phiError[2] = phiError[1];
        phiError[1] = phiError[0];

        thetaRef[4] = thetaRef[3];
        thetaRef[3] = thetaRef[2];
        thetaRef[2] = thetaRef[1];
        thetaRef[1] = thetaRef[0];

        ros::spinOnce();
        loop_rate.sleep();
    }
    //outfile.close();
    return 0;
}
