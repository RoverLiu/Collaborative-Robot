#include "arm_manager.h"
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include "order_handler.h"
#include "camera_handler.h"

arm_manager::arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
:_nh(nh),_nh_priv(nh_priv)
{

    left_arm = new robot_arm_control(nh, nh_priv, LEFT_PLANNING_GROUP, LEFT_GRIPPER_TOPIC);
    right_arm = new robot_arm_control(nh, nh_priv, RIGHT_PLANNING_GROUP, RIGHT_GRIPPER_TOPIC);
    my_camera = new camera_handler(nh, nh_priv);
    my_orders = new order_handler(nh, nh_priv);

    // set default position
    default_start_right_pos.position.x = 0.3500;
    default_start_right_pos.position.y = -0.25000;
    default_start_right_pos.position.z = 0.2000;
    default_start_right_pos.orientation = left_arm->get_direction(1);

    default_start_left_pos.position.x = 0.3500;
    default_start_left_pos.position.y = 0.25000;
    default_start_left_pos.position.z = 0.2000;
    default_start_left_pos.orientation = right_arm->get_direction(1);

    default_calibration_pos.position.x = 0.30000;
    default_calibration_pos.position.y = -0.00000;
    default_calibration_pos.position.z = 0.2000;
    default_calibration_pos.orientation = left_arm->get_direction(3);

    // reset
    left_arm->reset_arm_pos(left_arm_default_angle);
    right_arm->reset_arm_pos(right_arm_default_angle);

    // move to default position 
    std::cout<<"right pos"<<std::endl;
    right_arm->get_current_pose();
    // left_arm->auto_move_arm(default_start_left_pos);
    // left_arm->reset_arm_pos();
    // left_arm->reset_griper_direction();
    // left_arm->gripper_control(0);
    std::cout<<"left pos"<<std::endl;
    left_arm->get_current_pose();

    left_arm->auto_move_arm(default_start_left_pos);
    right_arm->auto_move_arm(default_start_right_pos);
 
    std::cout<<"-------------------reset finished--------------------"<<std::endl;
    

}

arm_manager::~arm_manager()
{
    delete left_arm;
    delete right_arm;
    delete my_camera;
    delete my_orders;
    delete left_arm_regression_x;
    delete left_arm_regression_y;
    delete right_arm_regression_x;
    delete right_arm_regression_y;
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
    // calibrate left arm
    // data to save
    std::vector<float> camera_x;
    std::vector<float> arm_x;
    std::vector<float> camera_y;
    std::vector<float> arm_y;

    // move to specific location
    geometry_msgs::Pose current_pos = default_start_left_pos;
    current_pos.orientation = left_arm->get_direction(3);

    // location 1
    current_pos.position.y -= calibration_gap;
    left_arm->auto_move_arm(current_pos);
    left_arm->gripper_control(0);
    ros::Duration(1).sleep();  // Sleep for 0.5 second
    std::vector<float> dat = my_camera->get_pos(my_camera->gripper_left);
    if (dat.empty()) 
    {
        ROS_INFO("location 1 error");
    }
    arm_x.push_back(current_pos.position.x);
    arm_y.push_back(current_pos.position.y);
    camera_x.push_back(dat.at(0));
    camera_y.push_back(dat.at(1));
    std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    ROS_INFO("location 1");
    // left_arm->auto_move_arm(default_start_left_pos);
    wait();

    // location 2
    current_pos.position.x += calibration_gap;
    left_arm->auto_move_arm(current_pos);
    ros::Duration(1).sleep();  // Sleep for 0.5 second
    dat = my_camera->get_pos(my_camera->gripper_left); 
    if (dat.empty()) 
    {
        ROS_INFO("location 2 error");
    }
    arm_x.push_back(current_pos.position.x);
    arm_y.push_back(current_pos.position.y);
    camera_x.push_back(dat.at(0));
    camera_y.push_back(dat.at(1));
    std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    ROS_INFO("location 2");
    wait();

    // location 3
    current_pos.position.x -= calibration_gap*2;
    left_arm->auto_move_arm(current_pos);
    ros::Duration(1).sleep();  // Sleep for 0.5 second
    dat = my_camera->get_pos(my_camera->gripper_left);
    if (dat.empty()) 
    {
        ROS_INFO("location 3 error");
    }
    arm_x.push_back(current_pos.position.x);
    arm_y.push_back(current_pos.position.y);
    camera_x.push_back(dat.at(0));
    camera_y.push_back(dat.at(1));
    std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    ROS_INFO("location 3");
    wait();

    // location 4
    current_pos.position.x += calibration_gap;
    current_pos.position.y -= 2*calibration_gap;
    left_arm->auto_move_arm(current_pos);
    ros::Duration(1).sleep();  // Sleep for 0.5 second
    dat = my_camera->get_pos(my_camera->gripper_left);
    if (dat.empty()) 
    {
        ROS_INFO("location 4 error");
    }
    arm_x.push_back(current_pos.position.x);
    arm_y.push_back(current_pos.position.y);
    camera_x.push_back(dat.at(0));
    camera_y.push_back(dat.at(1));
    std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    ROS_INFO("location 4");
    wait();
    
    // // location 5
    current_pos.position.y -= 2*calibration_gap;
    left_arm->auto_move_arm(current_pos);
    ros::Duration(1).sleep();  // Sleep for 0.5 second
    dat = my_camera->get_pos(my_camera->gripper_left);
    if (dat.empty()) 
    {
        ROS_INFO("location 5 error");
    }
    arm_x.push_back(current_pos.position.x);
    arm_y.push_back(current_pos.position.y);
    camera_x.push_back(dat.at(0));
    camera_y.push_back(dat.at(1));
    std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    ROS_INFO("location 5");
    wait();

    // move back
    left_arm->auto_move_arm(default_start_left_pos);
    left_arm->reset_griper_direction();

    // calculate
    left_arm_regression_x = new regression(camera_x, arm_x);
    left_arm_regression_y = new regression(camera_y, arm_y);
    left_arm_regression_x->PrintBestFittingLine();
    left_arm_regression_y->PrintBestFittingLine();

    wait();
    std::cout<<"right arm calibration start"<<std::endl;
    // clear vector and redo everything for right arm
    // arm_x.clear();
    // arm_y.clear();
    // camera_x.clear();
    // camera_y.clear();
    
    // // move to specific location
    // current_pos = default_start_right_pos;
    // current_pos.orientation = right_arm->get_direction(2);

    // // location 1
    // right_arm->auto_move_arm(current_pos);
    // right_arm->gripper_control(0);
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    // dat = my_camera->get_pos(my_camera->gripper_right);
    // if (dat.empty()) 
    // {
    //     ROS_INFO("location 1 error");
    // }
    // arm_x.push_back(current_pos.position.x);
    // arm_y.push_back(current_pos.position.y);
    // camera_x.push_back(dat.at(0));
    // camera_y.push_back(dat.at(1));
    // std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    // ROS_INFO("location 1");
    // // right_arm->auto_move_arm(default_start_left_pos);
    // wait();

    // // location 2
    // current_pos.position.x += calibration_gap;
    // right_arm->auto_move_arm(current_pos);
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    // dat = my_camera->get_pos(my_camera->gripper_right); 
    // if (dat.empty()) 
    // {
    //     ROS_INFO("location 2 error");
    // }
    // arm_x.push_back(current_pos.position.x);
    // arm_y.push_back(current_pos.position.y);
    // camera_x.push_back(dat.at(0));
    // camera_y.push_back(dat.at(1));
    // std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    // ROS_INFO("location 2");
    // wait();

    // // location 3
    // current_pos.position.x -= calibration_gap*2;
    // right_arm->auto_move_arm(current_pos);
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    // dat = my_camera->get_pos(my_camera->gripper_right);
    // if (dat.empty()) 
    // {
    //     ROS_INFO("location 3 error");
    // }
    // arm_x.push_back(current_pos.position.x);
    // arm_y.push_back(current_pos.position.y);
    // camera_x.push_back(dat.at(0));
    // camera_y.push_back(dat.at(1));
    // std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    // ROS_INFO("location 3");
    // wait();

    // // location 4
    // current_pos.position.x += calibration_gap;
    // current_pos.position.y -= 2*calibration_gap;
    // right_arm->auto_move_arm(current_pos);
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    // dat = my_camera->get_pos(my_camera->gripper_right);
    // if (dat.empty()) 
    // {
    //     ROS_INFO("location 4 error");
    // }
    // arm_x.push_back(current_pos.position.x);
    // arm_y.push_back(current_pos.position.y);
    // camera_x.push_back(dat.at(0));
    // camera_y.push_back(dat.at(1));
    // std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    // ROS_INFO("location 4");
    // wait();
    
    // // // location 5
    // current_pos.position.y -= 2*calibration_gap;
    // right_arm->auto_move_arm(current_pos);
    // ros::Duration(1).sleep();  // Sleep for 0.5 second
    // dat = my_camera->get_pos(my_camera->gripper_right);
    // if (dat.empty()) 
    // {
    //     ROS_INFO("location 5 error");
    // }
    // arm_x.push_back(current_pos.position.x);
    // arm_y.push_back(current_pos.position.y);
    // camera_x.push_back(dat.at(0));
    // camera_y.push_back(dat.at(1));
    // std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
    // ROS_INFO("location 5");
    // wait();


}



void arm_manager::pick_up_chocolate(int arm_num) 
{
    // get right arm to work
    robot_arm_control * my_arm = left_arm;
    regression * regression_x = left_arm_regression_x;
    regression * regression_y = left_arm_regression_y;
    if (arm_num == 1) {
        my_arm = right_arm;
        regression_x = right_arm_regression_x;
        regression_y = right_arm_regression_y;
    }

    // get order
    int chocolate_id = my_orders->get_chocolate_to_pick();

    if (chocolate_id == -1)
    {
        return;
    }

    // get pos
    std::vector<float> camera_pos = my_camera->get_pos(chocolate_id);
    while (camera_pos.empty()) 
    {
        ROS_INFO("chocolate not found ");
        camera_pos = my_camera->get_pos(chocolate_id);

        // todo: broadcast a message
    }

    // convert
    geometry_msgs::Pose arm_pos;
    arm_pos.orientation = my_arm->get_direction(1);
    arm_pos.position.x = regression_x->predict(camera_pos.at(0));
    arm_pos.position.y = regression_y->predict(camera_pos.at(1));
    arm_pos.position.z = default_chocolate_z_level;

    // pick it up
    my_arm->pick_up_and_delivery(arm_pos);

}
