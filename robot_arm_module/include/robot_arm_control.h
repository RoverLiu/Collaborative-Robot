/**
 * @file robot_arm_control.h
 * @author Rover
 * @brief Control robot arm with all basic functions
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ROBOT_ARM_CONTROL_H
#define __ROBOT_ARM_CONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <trajectory_msgs.h>
#include <iostream>
#include <string> // for string class

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>

class robot_arm_control 
{
    public:
        // methods
        robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv,  const std::string PLANNING_GROUP, const std::string gripper_topic);
        ~robot_arm_control();

        /**
         * @brief move arm to required position
         * 
         * @param goal the position to ge
         */
        void auto_move_arm( geometry_msgs::Pose goal);

        /**
         * @brief move arm with a straight line
         * 
         * @param waypoints a vector saves all changing angle to reach
         */
        void CartesianPath_move_arm( std::vector<geometry_msgs::Pose> waypoints);

        /**
         * @brief pick up the chocolate from a point and deliver it to the end position
         * 
         * @param goal The position to pick up the chocolate
         * @param end_pos THe position to drop the chocolate
         */
        void pick_up_and_delivery(geometry_msgs::Pose goal, geometry_msgs::Pose end_pos);

        /**
         * @brief Get the direction object
         * get orientation value for required direction 
         * @param direction (1: front, 2: left, 3: right)
         * @return geometry_msgs::Quaternion The orientation in desired form
         */
        geometry_msgs::Quaternion get_direction(int direction);

        /**
         * @brief open or close gripper 
         * 
         * @param state (0 means close, 1 means open)
         */
        void gripper_control( int state);

        /**
         * @brief Get the current pose object
         * 
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped get_current_pose();

        /**
         * @brief reset the direction of the gripper
         * 
         */
        void reset_griper_direction();

        /**
         * @brief Reset the position of the arm to desired angle
         * 
         * @param joint_group_positions The angle for each joint
         */
        void reset_arm_pos(std::vector<double> joint_group_positions);

        /**
         * @brief detach the stand from the world
         * 
         */
        void detach_stand_object();

        /**
         * @brief attach the stand from the world
         * 
         */
        void attach_stand_object();

    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // gripper control pub
        ros::Publisher gripper_pub;

        // choose planning group for the arm
        const std::string PLANNING_GROUP;

        // this is designed for yumi only
        // control gripper
        const std::string gripper_topic;

        // The :move_group_interface:`MoveGroupInterface` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface *move_group;

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup* joint_model_group;
        // const robot_state::JointModelGroup* right_joint_model_group;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        const double pi = 3.1415926535;

        // collision object
        moveit_msgs::CollisionObject table;
        moveit_msgs::CollisionObject stand;
};

#endif