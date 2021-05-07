#!/bin/sh
# 2017-11-29 LLW shell script for changing ownership and sticky bit
# usage: ~/bin/change_perms_encoder.sh
#
echo ls -l ~/catkin_ws/devel/lib/spring_break/encoderPublisher
ls -l ~/catkin_ws/devel/lib/spring_break/encoderPublisher

echo sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/encoderPublisher
sudo chown root:root  ~/catkin_ws/devel/lib/spring_break/encoderPublisher

echo sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/encoderPublisher
sudo chmod u+s  ~/catkin_ws/devel/lib/spring_break/encoderPublisher

echo ls -l ~/catkin_ws/devel/lib/spring_break/encoderPublisher
ls -l ~/catkin_ws/devel/lib/spring_break/encoderPublisher
