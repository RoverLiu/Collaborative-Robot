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
        robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv,  const std::string PLANNING_GROUP);
        ~robot_arm_control();

        // move arm to required position
        bool auto_move_arm(char arm_to_move, geometry_msgs::Pose goal);

        // move arm with a straight line
        bool CartesianPath_move_arm(char arm_to_move, std::vector<geometry_msgs::Pose> waypoints);

        // open or close gripper
        void gripper_control(char arm_to_move, char state);

    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        const std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroupInterface` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface *move_group;
        // moveit::planning_interface::MoveGroupInterface *right_move_group;

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup* joint_model_group;
        // const robot_state::JointModelGroup* right_joint_model_group;

        // testing methods
        void wait();


};

#endif