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
#include "order_handler.h"

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

        // these angles are set for robot arm itself
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

        // my defined position (beginning or ending position for each task)
        const std::vector<double> right_arm_finish_angle = {0.442606,
            -2.394394,
            -1.717076,
            0.384959,
            0.315967,
            0.872647,
            0.135105
        };

        const std::vector<double> left_arm_finish_angle = {-0.463526,
            -2.399707,
            1.691992,
            0.379761,
            -0.326887,
            0.879700,
            -0.141335
        };

        // default position for robot arm to start
        geometry_msgs::Pose default_start_right_pos;
        geometry_msgs::Pose default_start_left_pos;

        float default_start_right_pos_x = 0.1;
        float default_start_right_pos_y = 0.3;
        float default_start_right_pos_z = 0.25;

        // default position for robot arm to drop chocolate
        geometry_msgs::Pose default_drop_pos;

        float default_drop_pos_x = 0.6;
        float default_drop_pos_y = 0.00;
        float default_drop_pos_z = 0.25;

        // define which arm to pick (chocolate is on which side of the table)
        const float horizontal_threshold = 320.00;
        // calibration distance (compensate the size of gripper)
        const float X_CALIBRATION_LEFT = -0.150;
        const float X_CALIBRATION_RIGHT = -0.130;
        const float Y_CALIBRATION_LEFT = -0.055;
        const float Y_CALIBRATION_RIGHT = 0.06;
        // the distance difference between each position
        float calibration_gap = 0.05000;
        // height of chocolate
        float right_default_chocolate_z_level = 0.12000;
        float left_default_chocolate_z_level = 0.11000;




        



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
        order_handler * my_orders;

        // testing methods
        void wait();

        ~arm_manager();

        // void calibration_discard();
        void calibration();

        // pick up chocolate based on command saved in order_handler class
        void pick_up_chocolate();

        // load default calibrations
        void load_default_calibration();

};




#endif