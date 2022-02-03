#include "arm_manager.h"
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>

arm_manager::arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
:_nh(nh),_nh_priv(nh_priv)
{

    ros::AsyncSpinner spinner(1);
    spinner.start();

    left_arm = new robot_arm_control(nh, nh_priv, LEFT_PLANNING_GROUP);
    right_arm = new robot_arm_control(nh, nh_priv, RIGHT_PLANNING_GROUP);
}

arm_manager::~arm_manager()
{
    delete left_arm;
    delete right_arm;
}