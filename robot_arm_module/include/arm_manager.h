#ifndef __ARM_MANAGER_H
#define __ARM_MANAGER_H
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>

class arm_manager
{
private:
    /* data */
    // data
    // ROS NodeHandle
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv;

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    const std::string LEFT_PLANNING_GROUP = "left_arm";
    const std::string RIGHT_PLANNING_GROUP = "right_arm";

    robot_arm_control * left_arm;
    robot_arm_control * right_arm;


public:
    arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~arm_manager();
};







#endif