/**
 * @file robot_arm_control.cpp
 * @author Rover
 * @brief Control robot arm with all basic functions
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
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

/**
 * @brief Construct a new robot arm control::robot arm control object
 * 
 * @param nh Node handler for ros
 * @param nh_priv Previous node handler for ros
 * @param PLANNING_GROUP The name of the planning group in string
 * @param gripper_topic The topic name for the gripper control
 */
robot_arm_control::robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv, const std::string PLANNING_GROUP, const std::string gripper_topic)  
:_nh(nh),_nh_priv(nh_priv), PLANNING_GROUP(PLANNING_GROUP), gripper_topic(gripper_topic)
{
    /*------------------------------------------set up moveit---------------------------------------*/
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can get a list of all the groups in the robot:
    std::printf("Available Planning Groups:");
    std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
    
    // default
    move_group->setPlanningTime(30);
    move_group->setPlannerId("RRTConnectkConfigDefault");
    
    /*------------------------------------------set up griper control---------------------------------------*/
    gripper_pub = nh.advertise<std_msgs::Float64>(gripper_topic, 1000);

    //------------------------------------------set up objects avoidance-------------------------------------
    // set up table and stand
    table.header.frame_id = move_group->getPlanningFrame();
    stand.header.frame_id = move_group->getPlanningFrame();
    table.id = "table";
    stand.id = "chocolate_stand";
    shape_msgs::SolidPrimitive primitive;
    shape_msgs::SolidPrimitive stand_primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.2;

    stand_primitive.type = stand_primitive.BOX;
    stand_primitive.dimensions.resize(3);
    stand_primitive.dimensions[0] = 0.05;
    stand_primitive.dimensions[1] = 1.0;
    stand_primitive.dimensions[2] = 0.15;

    // position
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;

    geometry_msgs::Pose stand_pose;
    stand_pose.orientation.w = 1.0;
    stand_pose.position.x = 0.45;
    stand_pose.position.y = 0.0;
    stand_pose.position.z = 0.075;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(box_pose);
    table.operation = table.ADD;

    stand.primitives.push_back(stand_primitive);
    stand.primitive_poses.push_back(stand_pose);
    stand.operation = stand.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table);
    collision_objects.push_back(stand);
    
    ROS_INFO( "Add objects into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}

/**
 * @brief Destroy the robot arm control::robot arm control object
 * 
 */
robot_arm_control::~robot_arm_control() 
{
    delete move_group;
}

/**
 * @brief move to the given position
 * +x 向前移动
 * +y 像左侧桌子移动
 * @param goal The position to reach
 */
void robot_arm_control::auto_move_arm( geometry_msgs::Pose goal) 
{
    // print position to reach
    // std::cout<<goal.position<<std::endl;
    
    // set 
    move_group->setPoseTarget(goal);

    // plan
    move_group->plan(my_plan);

    // execuate
    move_group->move();
}

/**
 * @brief move arm with a straight line
 * 
 * @param waypoints a vector saves all changing angle to reach
 */
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
 * @param direction 1: front, 2: left, 3: right, 4 down, 5 up, 6 drop
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
            myQuaternion.setRPY( pi/2, pi/2, 0 ); 
            break; //optional

        case 4  :
            myQuaternion.setRPY( 0, pi, 0 ); 
            break; //optional

        case 5  :
            myQuaternion.setRPY( 0, 0, 0 ); 
            break; //optional
        
        case 6  :
            myQuaternion.setRPY( pi/2, 0, pi/2 ); 
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

/**
 * @brief get current position and orientation details
 * 
 * @return geometry_msgs::PoseStamped The current pos details
 */
geometry_msgs::PoseStamped robot_arm_control::get_current_pose() 
{
    geometry_msgs::PoseStamped current_pos;

    current_pos = move_group->getCurrentPose();

    // print out current pos
    // std::cout<<current_pos.pose.position<<std::endl;
    // std::cout<<current_pos.pose.orientation<<std::endl;

    // get angel detail
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // print the angle out
    // printf("Pos detail in angle:\n");
    // for (double i : joint_group_positions) 
    // {
    //     printf("%f\n", i);
    // }

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
}

/**
 * @brief Reset arm to required posture with given angle
 * 
 * @param joint_group_positions THe angle degree for each joint
 */
void robot_arm_control::reset_arm_pos(std::vector<double> joint_group_positions) 
{
    move_group->setJointValueTarget(joint_group_positions);
    move_group->plan(my_plan);
    move_group->move();
}

/**
 * @brief pick up the chocolate from a point and deliver it to the end position
 * 
 * @param goal The position to pick up the chocolate
 * @param end_pos THe position to drop the chocolate
 */
void robot_arm_control::pick_up_and_delivery(geometry_msgs::Pose goal, geometry_msgs::Pose end_pos) 
{
    ros::Duration(3).sleep();  // Sleep for 0.5 second
    // open gripper
    gripper_control(1);
    reset_griper_direction();
    
    // get current pos
    geometry_msgs::PoseStamped current_detail = get_current_pose();
    geometry_msgs::Pose current_pos = current_detail.pose;

    // generate the path
    std::vector<geometry_msgs::Pose> waypoints;

    // go to that pos
    current_pos.position.z = goal.position.z;
    waypoints.push_back(current_pos);

    current_pos.position.y = goal.position.y;
    waypoints.push_back(current_pos);

    current_pos.position.x = goal.position.x;
    waypoints.push_back(current_pos);

    CartesianPath_move_arm(waypoints);
    ros::Duration(3.0).sleep();

    // pick up
    gripper_control(0);
    ros::Duration(2.0).sleep();

    // delivery
    waypoints.clear();

    std::cout<<"X: "<<end_pos.position.x<<" Y: "<<end_pos.position.y<<" Z: "<<end_pos.position.z<<std::endl;
    current_pos.position.z = end_pos.position.z;
    waypoints.push_back(current_pos);

    // follow horizontal and vertial line
    // current_pos.position.x = end_pos.position.x;
    // waypoints.push_back(current_pos);

    // current_pos.position.y = end_pos.position.y;
    // waypoints.push_back(current_pos);

    // go directly
    current_pos.position.x = end_pos.position.x;
    current_pos.position.y = end_pos.position.y;
    waypoints.push_back(current_pos);

    CartesianPath_move_arm(waypoints);

    ros::Duration(3.0).sleep();  // Sleep for 0.5 second

    gripper_control(1);
}

/**
 * @brief detach the stand from the world
 * 
 */
void robot_arm_control::detach_stand_object()
{
    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(stand.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
}

/**
 * @brief attach the stand from the world
 * 
 */
void robot_arm_control::attach_stand_object()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(stand);
    planning_scene_interface.addCollisionObjects(collision_objects);
}