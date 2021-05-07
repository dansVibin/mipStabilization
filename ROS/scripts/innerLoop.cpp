#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <rc/button.h>

extern "C"
{
#include <robotcontrol.h>
}

bool startPrgm = false;
int bttnCount = 0;
int state = 1;
static void __on_mode_press(void)
{
   printf("Mode Pressed\n");
   return;
}
static void __on_mode_release(void)
{
   printf("Mode Released\n");
   bttnCount += state;

   if (bttnCount == 0){
      startPrgm = false;
      state = 1;
   }
   if (bttnCount == 1){
      startPrgm = true;
      state = -1;
   }
   return;
}

double duty [5] = {0.0, 0.0, 0.0 , 0.0 , 0.0};//{k,k-1,k-2}
int ch [3] = {0 , 1 , 2}; //motor channel
int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
double sampleRate = 100;
double dt = 1/sampleRate;
double timeCount = 0;
std::ofstream outfile ("innerLoopLog.txt");

//Dz1 controller = (b2*z^2 + b1*z + b0)/(z^2 + a1*z + a0)
//100Hz
double Dz1num [5] = {-3.125, 5.173, 3.281, -3.486, -2.157}; //{b0,b1,b2}
double Dz1den [4] = {0.1777, 0.007581, -0.8703, -0.3149}; //{a0,a1}
double P = 0.8770;

double thetaRef = 0.0;
double thetaMeasured = 0.0;
double offset = 0.0;
double thetaError [5] = {0,0,0,0,0}; //{k,k-1,k-2}

void thetaRefCallback(const std_msgs::Float64::ConstPtr & msg){
    if(startPrgm == true){
       thetaRef = P*(msg->data);//multiply reference theta by prescalar
    }
}

void bodyAngleCallback(const std_msgs::Float64::ConstPtr & msg){
    if(startPrgm == true){//button press starts program
       thetaMeasured = (msg -> data) + offset;

       thetaError[0] = thetaRef - thetaMeasured;

       //U(k) difference equation
       //duty[0] = Dz1num[2]*thetaError[0] + Dz1num[1]*thetaError[1] + Dz1num[0]*thetaError[2] - Dz1den[1]*duty[1] - Dz1den[0]*duty[2];
duty[0] =Dz1num[4]*thetaError[0] +Dz1num[3]*thetaError[1] +Dz1num[2]*thetaError[2] +Dz1num[1]*thetaError[3] +Dz1num[0]*thetaError[4]  -Dz1den[3]*duty[1] - Dz1den[2]*duty[2] - Dz1den[1]*duty[3] - Dz1den[0]*duty[4];

       //saturate duty cycle
       if(duty[0] > 1) duty[0] = 1;
       else if (duty[0] < -1) duty[0] = -1;

       //asign duty cycle to motors
       if(thetaMeasured > 1.25 || thetaMeasured < -0.88){
           rc_motor_set(ch[1], 0);
           rc_motor_set(ch[2], 0);
       }
       else{
           rc_motor_set(ch[1],duty[0]);
           rc_motor_set(ch[2],duty[0]);
       }
       //store previous duty values
       duty[4] = duty[3];
       duty[3] = duty[2];
       duty[2] = duty[1];
       duty[1] = duty[0];

       //output log to txt file
       outfile << timeCount << " " << thetaRef << " " << thetaMeasured << " " << thetaError[0] << " " << duty[0] << std::endl;
       timeCount += dt;

       //store previous thetaError values
       thetaError[4] = thetaError[3];
       thetaError[3] = thetaError[2];
       thetaError[2] = thetaError[1];
       thetaError[1] = thetaError[0];

   //print results
   //ROS_INFO("%f , %f , %f , %f, %f",offset ,thetaRef, thetaMeasured, thetaError[0], duty[0]);
   }

   //set offset to 'zero out' theta
   else{
   rc_motor_set(ch[1],0);
   rc_motor_set(ch[2],0);
   offset = 0.0 - (msg -> data);
   thetaRef = 0.0;
   thetaMeasured = 0.0;
   double thetaError [5] = {0,0,0,0,0}; //{k,k-1,k-2}
   }
}

int main(int argc,char ** argv)
{
    //initialize beaglebone buttons
    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
                                        RC_BTN_DEBOUNCE_DEFAULT_US)){
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, __on_mode_release);

    // initialize motor hardware
    if(rc_motor_init_freq(freq_hz)) return -1;

    //set ros node
    ros::init(argc,argv,"innerloop");
    ros::NodeHandle inner;

    //subscribe to body angle and theta reference angle
    ros::Subscriber theta_sub = inner.subscribe("body_angle",1,bodyAngleCallback);
    ros::Subscriber ref_sub = inner.subscribe("theta_reference",1,thetaRefCallback);

    //outfile << Dz1num[0] << " " << Dz1num[1] << " " << Dz1num[2] << " " << Dz1den[0] << " " << Dz1den[1] << std::endl;

    ros::Rate loop_rate(sampleRate);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    outfile.close();
    rc_motor_cleanup();
    return 0;
}
