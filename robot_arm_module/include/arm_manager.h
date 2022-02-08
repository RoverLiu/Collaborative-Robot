#ifndef __ARM_MANAGER_H
#define __ARM_MANAGER_H
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include "camera_handler.h"
#include "regression.h"

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

        const std::string LEFT_GRIPPER_TOPIC = "/yumi/gripper_l_effort_cmd";
        const std::string RIGHT_GRIPPER_TOPIC = "/yumi/gripper_r_effort_cmd";

        const std::vector<double> right_arm_default_angle = {-0.000103,
            -2.268631,
            -2.356222,
            0.523748,
            -0.000088,
            0.702391,
            -0.000187
        };

        const std::vector<double> left_arm_default_angle = {0.000144,
            -2.268888,
            2.356080,
            0.523852,
            0.000080,
            0.698850,
            -0.000160
        };

        // default position for robot arm
        geometry_msgs::Pose default_start_right_pos;
        geometry_msgs::Pose default_start_left_pos;
        geometry_msgs::Pose default_calibration_pos;

        float calibration_gap = 0.05000;

        // save transform method
        regression * left_arm_regression_x;
        regression * left_arm_regression_y;
        regression * right_arm_regression_x;
        regression * right_arm_regression_y;


    public:
        arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv);

        robot_arm_control * left_arm;
        robot_arm_control * right_arm;

        camera_handler * my_camera;

        // testing methods
        void wait();

        ~arm_manager();

        void calibration();

};







#endif