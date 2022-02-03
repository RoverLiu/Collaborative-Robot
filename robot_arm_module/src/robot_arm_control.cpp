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
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Float64.h>

robot_arm_control::robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv, const std::string PLANNING_GROUP, const std::string gripper_topic)  
:_nh(nh),_nh_priv(nh_priv), PLANNING_GROUP(PLANNING_GROUP), gripper_topic(gripper_topic)
{
    /*------------------------------------------set up moveit---------------------------------------*/
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    // right_move_group = new moveit::planning_interface::MoveGroupInterface(RIGHT_PLANNING_GROUP);
    joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // right_joint_model_group =  right_move_group->getCurrentState()->getJointModelGroup(RIGHT_PLANNING_GROUP);

    ROS_INFO("%s Planning frame: %s", PLANNING_GROUP, move_group->getPlanningFrame().c_str());
    // ROS_INFO_NAMED("tutorial", "Right Planning frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("%s End effector link: %s", PLANNING_GROUP, move_group->getEndEffectorLink().c_str());
    // ROS_INFO_NAMED("tutorial", "Right End effector link: %s", right_move_group->getEndEffectorLink().c_str());

    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //             std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO("tutorial", "Planning frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO("tutorial", "Available Planning Groups:");
    std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
    
    // default
    move_group->setPlanningTime(10);
    // right_move_group->setPlanningTime(10);
    move_group->setPlannerId("RRTConnectkConfigDefault");

    // move_group->setMaxVelocityScalingFactor(0.5);

    // /*------------------------------------------visualization---------------------------------------*/
    // visual_tools.deleteAllMarkers();
    // visual_tools.loadRemoteControl();
    
    /*------------------------------------------set up griper control---------------------------------------*/
    gripper_pub = nh.advertise<std_msgs::Float64>(gripper_topic, 1000);
    
}

robot_arm_control::~robot_arm_control() 
{
    delete move_group;
    // delete right_move_group;
}



bool robot_arm_control::auto_move_arm( geometry_msgs::Pose goal) 
{
    ROS_INFO("enter");
    std::cout<<goal.position<<std::endl;
    
    move_group->setPoseTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("enter2");

    move_group->plan(my_plan);
    // bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO("Move to position plannning result %s \n", success ? "" : "FAILED");
    ROS_INFO("enter3");

    move_group->move();

    ROS_INFO("enter3");

    return true;
}

void robot_arm_control::CartesianPath_move_arm( std::vector<geometry_msgs::Pose> waypoints) 
{
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Trajectory Movement (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    
    move_group->move();

}

/**
 * @brief return w value for differetn direction
 * 
 * @param direction 1: front, 2: left, 3: right,
 * @return geometry_msgs::Quaternion 
 */
geometry_msgs::Quaternion robot_arm_control::get_direction(int direction)
{
    geometry_msgs::Quaternion orientation;
    // orientation.x = 0.00000;
    // orientation.y = 0.00000;
    // orientation.z = 1.00000;

    switch(direction) {
        case 1  :
            orientation.w = 0.00000;
            break; //optional
        case 2  :
            orientation.w = 1.57080;
            break; //optional
        case 3  :
            orientation.w = -1.57080;
            break; //optional

        // you can have any number of case statements.
        default : //Optional
            orientation.w = 0.00000;        
    }

    return orientation;
}

/**
 * @brief open/close gripper
 * 
 * @param state 0 means close, 1 means open
 */
void robot_arm_control::gripper_control( int state)
{
    // for now, the position of gripper is ranging between -20 to 20
    std_msgs::Float64 pos;
    if (state == 0) 
    {
        pos.data = 20;
        gripper_pub.publish(pos);
    }
    else
    {
        pos.data = -20;
        gripper_pub.publish(pos);
    }
    
}