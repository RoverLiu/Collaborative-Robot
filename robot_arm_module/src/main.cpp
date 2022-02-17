/**
 * @file main.cpp
 * @author Rover
 * @brief main file for the robot control, and chocolate pick up
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "arm_manager.h"
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "camera_handler.h"
#include "regression.h"
#include "order_handler.h"

int main(int argc, char* argv[]) {
    // Initialize the ros
    ros::init( argc, argv, "arm_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );
    // ROS spin
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // initialize my manager
    arm_manager my_manager(nh, nh_priv);

    // choose between calibration or default values
    // my_manager.calibration();
    my_manager.load_default_calibration();

    // loop for the chocolate pick up
    while (true) 
    {
        my_manager.pick_up_chocolate();
    }

}