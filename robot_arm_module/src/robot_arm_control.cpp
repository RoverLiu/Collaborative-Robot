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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

robot_arm_control::robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv, const std::string PLANNING_GROUP, const std::string gripper_topic)  
:_nh(nh),_nh_priv(nh_priv), PLANNING_GROUP(PLANNING_GROUP), gripper_topic(gripper_topic)
{
    /*------------------------------------------set up moveit---------------------------------------*/
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    // right_move_group = new moveit::planning_interface::MoveGroupInterface(RIGHT_PLANNING_GROUP);
    joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // right_joint_model_group =  right_move_group->getCurrentState()->getJointModelGroup(RIGHT_PLANNING_GROUP);

    // std::printf("%s Planning frame: %s", PLANNING_GROUP,  move_group->getPlanningFrame().c_str());
    // ROS_INFO_NAMED("tutorial", "Right Planning frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    // std::printf("%s End effector link: %s", PLANNING_GROUP,  move_group->getEndEffectorLink().c_str());
    // ROS_INFO_NAMED("tutorial", "Right End effector link: %s", right_move_group->getEndEffectorLink().c_str());

    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //             std::ostream_iterator<std::string>(std::cout, ", "));

    // std::printf("Planning frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    // std::printf("End effector link: %s", move_group->getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    std::printf("Available Planning Groups:");
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


// +x 向前移动
// +y 像左侧桌子移动
bool robot_arm_control::auto_move_arm( geometry_msgs::Pose goal) 
{
    // ROS_INFO("enter");
    std::cout<<goal.position<<std::endl;
    
    move_group->setPoseTarget(goal);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // ROS_INFO("enter2");

    move_group->plan(my_plan);
    // bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO("Move to position plannning result %s \n", success ? "" : "FAILED");
    // ROS_INFO("enter3");

    move_group->move();

    // ROS_INFO("enter3");

    return true;
}

void robot_arm_control::CartesianPath_move_arm( std::vector<geometry_msgs::Pose> waypoints) 
{
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(0.1);


    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    // moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Trajectory Movement (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    move_group->execute(my_plan);

}

/**
 * @brief return w value for differetn direction
 * 
 * @param direction 1: front, 2: left, 3: right, 4 up, 5 down
 * @return geometry_msgs::Quaternion 
 */
geometry_msgs::Quaternion robot_arm_control::get_direction(int direction)
{
    geometry_msgs::Quaternion orientation;

    // get use tf to calculate orientation
    tf2::Quaternion myQuaternion;
    // tf2::Quaternion target_direction;

    switch(direction) {
        case 1  :
            myQuaternion.setRPY( 0, pi/2, 0 ); 
            break; //optional
        case 2  :
            myQuaternion.setRPY( -pi/2, pi/2, 0 ); 
            break; //optional
        case 3  :
            myQuaternion.setRPY( pi/2, -pi/2, 0 ); 
            break; //optional

        case 4  :
            myQuaternion.setRPY( 0, 0, 0 ); 
            break; //optional

        case 5  :
            myQuaternion.setRPY( 0, pi, 0 ); 
            break; //optional

        // you can have any number of case statements.
        default : //Optional
            myQuaternion.setRPY( 0, pi/2, 0 ); 
    }

    myQuaternion.normalize();
    
    tf2::convert(myQuaternion, orientation);


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

geometry_msgs::PoseStamped robot_arm_control::get_current_pose() 
{
    geometry_msgs::PoseStamped current_pos;

    current_pos = move_group->getCurrentPose();

    std::cout<<current_pos.pose.position<<std::endl;
    std::cout<<current_pos.pose.orientation<<std::endl;

    // get angel detail
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    printf("Pos detail in angle:\n");
    for (double i : joint_group_positions) 
    {
        printf("%f\n", i);
    }

    return current_pos;
}

/**
 * @brief reset the heading direction of gripper
 * 
 */
void robot_arm_control::reset_griper_direction() 
{
    // get current pose
    geometry_msgs::PoseStamped current_pose = get_current_pose();
    geometry_msgs::Pose target_pose = current_pose.pose;
    target_pose.orientation = get_direction(1);

    auto_move_arm(target_pose);
    // get_current_pose();
}

void robot_arm_control::reset_arm_pos(std::vector<double> joint_group_positions) 
{

    move_group->setJointValueTarget(joint_group_positions);
    move_group->plan(my_plan);

    // // get current pose
    // geometry_msgs::PoseStamped current_pose = get_current_pose();
    // geometry_msgs::Pose target_pose = current_pose.pose;
    // target_pose.orientation = get_direction(1);

    move_group->move();
}

/**
 * @brief pick up the chocolate at given location
 * 
 * @param goal 
 */
void robot_arm_control::pick_up_and_delivery(geometry_msgs::Pose goal) 
{
    ros::Duration(3).sleep();  // Sleep for 0.5 second
    // open gripper
    gripper_control(1);
    reset_griper_direction();
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    

    // get current pos
    geometry_msgs::PoseStamped current_detail = get_current_pose();
    geometry_msgs::Pose current_pos = current_detail.pose;

    // generate the path
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(current_pos);

    current_pos.position.z = goal.position.z;
    waypoints.push_back(current_pos);

    current_pos.position.y = goal.position.y;
    waypoints.push_back(current_pos);

    current_pos.position.x = goal.position.x;
    waypoints.push_back(current_pos);

    CartesianPath_move_arm(waypoints);

    // pick up
    gripper_control(0);

    ros::Duration(1.0).sleep();


    // delivery
    waypoints.clear();
    // waypoints.push_back(current_pos);

    current_pos.position.z = 0.25000;
    waypoints.push_back(current_pos);

    current_pos.position.x = 0.3000;
    waypoints.push_back(current_pos);

    current_pos.position.y = 0.3000;
    waypoints.push_back(current_pos);

    CartesianPath_move_arm(waypoints);

    ros::Duration(1).sleep();  // Sleep for 0.5 second

    gripper_control(1);


    
    //  move 
}