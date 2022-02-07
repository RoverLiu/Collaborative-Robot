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
    my_camera = new camera_handler(nh, nh_priv);

    // set default position
    default_start_right_pos.position.x = 0.15000;
    default_start_right_pos.position.y = 0.20000;
    default_start_right_pos.position.z = 0.15000;

    default_start_left_pos.position.x = 0.15000;
    default_start_left_pos.position.y = -0.20000;
    default_start_left_pos.position.z = 0.15000;

    default_calibration_pos.position.x = 0.10000;
    default_calibration_pos.position.y = -0.00000;
    default_calibration_pos.position.z = 0.15000;

}

arm_manager::~arm_manager()
{
    delete left_arm;
    delete right_arm;
    delete my_camera;
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

void arm_manager::calibration() {
    


}