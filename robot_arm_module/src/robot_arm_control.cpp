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

robot_arm_control::robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv, const std::string PLANNING_GROUP)  
:_nh(nh),_nh_priv(nh_priv), PLANNING_GROUP(PLANNING_GROUP)
{

    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    // right_move_group = new moveit::planning_interface::MoveGroupInterface(RIGHT_PLANNING_GROUP);
    joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // right_joint_model_group =  right_move_group->getCurrentState()->getJointModelGroup(RIGHT_PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "%s Planning frame: %s", PLANNING_GROUP, move_group->getPlanningFrame().c_str());
    // ROS_INFO_NAMED("tutorial", "Right Planning frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "%s Left End effector link: %s", PLANNING_GROUP, move_group->getEndEffectorLink().c_str());
    // ROS_INFO_NAMED("tutorial", "Right End effector link: %s", right_move_group->getEndEffectorLink().c_str());

    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //             std::ostream_iterator<std::string>(std::cout, ", "));


    move_group->setPlanningTime(10);
    // right_move_group->setPlanningTime(10);
    move_group->setPlannerId("RRTConnectkConfigDefault");
    // right_move_group->setPlannerId("RRTConnectkConfigDefault");
}

robot_arm_control::~robot_arm_control() 
{
    delete move_group;
    // delete right_move_group;

}

void robot_arm_control::wait() {
  std::cout<<"Wait to continue, enter anything"<<std::endl;
  std::string a;
  std::cin >> a;
}