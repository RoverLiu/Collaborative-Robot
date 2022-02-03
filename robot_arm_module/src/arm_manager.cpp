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

    left_arm = new robot_arm_control(nh, nh_priv, LEFT_PLANNING_GROUP, LEFT_GRIPPER_TOPIC);
    right_arm = new robot_arm_control(nh, nh_priv, RIGHT_PLANNING_GROUP, RIGHT_GRIPPER_TOPIC);
}

arm_manager::~arm_manager()
{
    delete left_arm;
    delete right_arm;
}

/**
 * @brief wait until a input from keyboard
 * 
 */
void arm_manager::wait() 
{
    std::cout<<"Wait to continue, enter anything"<<std::endl;
    std::string a;
    std::cin >> a;
}