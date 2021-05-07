#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit
# usage: ~/bin/change_perms_servo.sh
#
echo ls -l ~/catkin_ws/devel/lib/spring_break/moveServo
ls -l ~/catkin_ws/devel/lib/spring_break/moveServo

echo sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/moveServo
sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/moveServo

echo sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/moveServo
sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/moveServo

echo ls -l ~/catkin_ws/devel/lib/spring_break/moveServo
ls -l ~/catkin_ws/devel/lib/spring_break/moveServo