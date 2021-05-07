#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit
# usage: ~/bin/change_perms_encoderTEST.sh
#
echo ls -l ~/catkin_ws/devel/lib/spring_break/encoderTEST
ls -l ~/catkin_ws/devel/lib/spring_break/encoderTEST

echo sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/encoderTEST
sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/encoderTEST

echo sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/encoderTEST
sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/encoderTEST

echo ls -l ~/catkin_ws/devel/lib/spring_break/encoderTEST
ls -l ~/catkin_ws/devel/lib/spring_break/encoderTEST