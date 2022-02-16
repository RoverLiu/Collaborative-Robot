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
#include "play_audio.h"


arm_manager::arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
:_nh(nh),_nh_priv(nh_priv)
{

    left_arm = new robot_arm_control(nh, nh_priv, LEFT_PLANNING_GROUP, LEFT_GRIPPER_TOPIC);
    right_arm = new robot_arm_control(nh, nh_priv, RIGHT_PLANNING_GROUP, RIGHT_GRIPPER_TOPIC);
    my_camera = new camera_handler(nh, nh_priv);
    my_orders = new order_handler(nh, nh_priv);

    // set default position
    // roughly +- 0.6 is the maximum distance it cango
    default_start_right_pos.position.x = default_start_right_pos_x;
    default_start_right_pos.position.y = -default_start_right_pos_y;
    default_start_right_pos.position.z = default_start_right_pos_z;
    default_start_right_pos.orientation = left_arm->get_direction(1);

    default_start_left_pos.position.x = default_start_right_pos_x;
    default_start_left_pos.position.y = default_start_right_pos_y;
    default_start_left_pos.position.z = default_start_right_pos_z;
    default_start_left_pos.orientation = right_arm->get_direction(1);

    // define drop position
    default_drop_pos.position.x = default_drop_pos_x;
    default_drop_pos.position.y = default_drop_pos_y;
    default_drop_pos.position.z = default_drop_pos_z;
    default_drop_pos.orientation = right_arm->get_direction(6);

    // default_calibration_pos.position.x = 0.30000;
    // default_calibration_pos.position.y = -0.00000;
    // default_calibration_pos.position.z = 0.2000;
    // default_calibration_pos.orientation = left_arm->get_direction(3);

    // left_arm->auto_move_arm(default_drop_pos);
    // left_arm->get_current_pose();
    // wait();

    // reset
    left_arm->reset_arm_pos(left_arm_default_angle);
    right_arm->reset_arm_pos(right_arm_default_angle);

    // // get current pos
    
    left_arm->auto_move_arm(default_start_left_pos);
    right_arm->auto_move_arm(default_start_right_pos);
    // left_arm->get_current_pose();
    // right_arm->get_current_pose();

    // wait();


    // // move to default position 
    // std::cout<<"right pos"<<std::endl;
    // right_arm->get_current_pose();
    // // left_arm->auto_move_arm(default_start_left_pos);
    // // left_arm->reset_arm_pos();
    // // left_arm->reset_griper_direction();
    // // left_arm->gripper_control(0);
    // std::cout<<"left pos"<<std::endl;
    // left_arm->get_current_pose();

    left_arm->reset_arm_pos(left_arm_finish_angle);
    right_arm->reset_arm_pos(right_arm_finish_angle);

    // move to default position 
    std::cout<<"right pos default angle"<<std::endl;
    right_arm->get_current_pose();
    // left_arm->auto_move_arm(default_start_left_pos);
    // left_arm->reset_arm_pos();
    // left_arm->reset_griper_direction();
    // left_arm->gripper_control(0);
    std::cout<<"left pos default angle"<<std::endl;
    left_arm->get_current_pose();
 
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
 * @brief load default calibration values
 * 
 */
void arm_manager::load_default_calibration()
{
    std::cout<<"Load default calibration values. Calibration is recommended for better accuracy"<<std::endl;
    left_arm_regression_x = new regression(0.00120, 0.051830);
    left_arm_regression_y = new regression(0.00129, -0.366111);
    right_arm_regression_x = new regression(0.00124, 0.043563);
    right_arm_regression_y = new regression(0.00133, -0.511030);
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
    // detach the stand
    left_arm->detach_stand_object();
    right_arm->detach_stand_object();

    MsgSpeakOut("calibration-start.wav");
    // calibrate left arm
    // data to save
    std::vector<float> camera_x;
    std::vector<float> arm_x;
    std::vector<float> camera_y;
    std::vector<float> arm_y;

    // move to specific location
    geometry_msgs::Pose current_pos = default_start_left_pos;
    current_pos.position.x += 3.5*calibration_gap;
    current_pos.position.y -= calibration_gap;
    current_pos.position.z -= 2*calibration_gap;
    current_pos.orientation = left_arm->get_direction(3);
    left_arm->gripper_control(0);
    std::vector<float> dat;


    std::cout<<"left arm calibration start"<<std::endl;
    float my_gap = calibration_gap;
    for (int i = 0; i < 4; i ++)
    {
        for (int j = 0; j < 4; j++) 
        {
            // move
            left_arm->auto_move_arm(current_pos);
            ros::Duration(1.0).sleep();  // Sleep for 0.5 second

            // read and update
            dat = my_camera->get_pos(my_camera->gripper_left); 
            if (dat.empty()) 
            {
                std::cout<<"position: i: "<<i<<" j: "<<j<<" failed!!!!!!!!!!!!!!"<<std::endl;
                continue;
            }
            arm_x.push_back(current_pos.position.x);
            arm_y.push_back(current_pos.position.y);
            camera_x.push_back(dat.at(0));
            camera_y.push_back(dat.at(1));
            std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
            // left_arm->get_current_pose();

            // next pos
            current_pos.position.x += my_gap;
        }
        current_pos.position.y -= calibration_gap;
        my_gap = -my_gap;

    }

    // move back
    left_arm->reset_arm_pos(left_arm_finish_angle);
    left_arm->reset_griper_direction();

    // calculate
    left_arm_regression_x = new regression(camera_y, arm_x);
    left_arm_regression_y = new regression(camera_x, arm_y);
    left_arm_regression_x->PrintBestFittingLine();
    left_arm_regression_y->PrintBestFittingLine();

    std::cout<<"right arm calibration start"<<std::endl;
    // clear vector and redo everything for right arm
    arm_x.clear();
    arm_y.clear();
    camera_x.clear();
    camera_y.clear();
    
    // move to specific location
    current_pos = default_start_right_pos;
    current_pos.orientation = right_arm->get_direction(2);
    right_arm->gripper_control(0);
    current_pos.position.x += 3.5*calibration_gap;
    current_pos.position.y += calibration_gap;
    current_pos.position.z -= 2*calibration_gap;

    my_gap = calibration_gap;
    for (int i = 0; i < 4; i ++)
    {
        for (int j = 0; j < 4; j++) 
        {
            // move
            right_arm->auto_move_arm(current_pos);
            ros::Duration(1.0).sleep();  // Sleep for 0.5 second

            // read and update
            dat = my_camera->get_pos(my_camera->gripper_right); 
            if (dat.empty()) 
            {
                std::cout<<"position: i: "<<i<<" j: "<<j<<" failed!!!!!!!!!!!!!!!"<<std::endl;
                continue;
            }
            arm_x.push_back(current_pos.position.x);
            arm_y.push_back(current_pos.position.y);
            camera_x.push_back(dat.at(0));
            camera_y.push_back(dat.at(1));
            std::cout<<"x: "<< dat.at(0)<<" y: "<<dat.at(1)<<std::endl;
            // right_arm->get_current_pose();

            // next pos
            current_pos.position.x += my_gap;
        }
        current_pos.position.y += calibration_gap;
        my_gap = -my_gap;

    }

    // move back
    right_arm->reset_arm_pos(right_arm_finish_angle);
    right_arm->reset_griper_direction();

    // calculate
    right_arm_regression_x = new regression(camera_y, arm_x);
    right_arm_regression_y = new regression(camera_x, arm_y);
    right_arm_regression_x->PrintBestFittingLine();
    right_arm_regression_y->PrintBestFittingLine();

    std::cout<<"Calibration done!"<<std::endl;
//     // wait();

    // add object
    // detach the stand
    left_arm->attach_stand_object();
    right_arm->attach_stand_object();

    MsgSpeakOut("calibration-finish.wav");

}




void arm_manager::pick_up_chocolate() 
{

    // get order
    int chocolate_id = my_orders->get_chocolate_to_pick();
    // my_orders->print_data();

    if (chocolate_id == -1)
    {
        // wait();
        // ROS_INFO("Wait for new command");
        return;
    }

    // get pos
    std::vector<float> camera_pos;
    camera_pos = my_camera->get_pos(chocolate_id);

    if (camera_pos.empty()) 
    {
        ROS_INFO("chocolate not found ");
        MsgSpeakOut("choose-another-one.wav");

        // camera_pos = my_camera->get_pos(chocolate_id);
        return;
        // todo: broadcast a message
    }

    std::cout<<"here is the chocolate ID required: "<<chocolate_id<<std::endl;


    // wait();

    // get right arm to work
    robot_arm_control * my_arm = left_arm;
    geometry_msgs::Pose end_pos = default_drop_pos;
    std::vector<double> default_pos = left_arm_finish_angle;
    regression * regression_x = left_arm_regression_x;
    regression * regression_y = left_arm_regression_y;
    float Y_CALIBRATION = Y_CALIBRATION_LEFT;
    float X_CALIBRATION = X_CALIBRATION_LEFT;
    float default_chocolate_z_level = left_default_chocolate_z_level;
    if (camera_pos.at(0) < horizontal_threshold) {
        my_arm = right_arm;
        // end_pos = default_start_right_pos;
        default_pos = right_arm_finish_angle;
        regression_x = right_arm_regression_x;
        regression_y = right_arm_regression_y;
        Y_CALIBRATION = Y_CALIBRATION_RIGHT;
        default_chocolate_z_level = right_default_chocolate_z_level;
        X_CALIBRATION = X_CALIBRATION_RIGHT;
    }




    // convert
    geometry_msgs::Pose arm_pos;
    arm_pos.orientation = my_arm->get_direction(1);
    arm_pos.position.x = regression_x->predict(camera_pos.at(1)) + X_CALIBRATION;
    arm_pos.position.y = regression_y->predict(camera_pos.at(0)) + Y_CALIBRATION;
    arm_pos.position.z = default_chocolate_z_level;
    
    regression_x->PrintBestFittingLine();
    regression_y->PrintBestFittingLine();
    std::cout<<"x: "<<arm_pos.position.x
    <<" y: "<<arm_pos.position.y<<std::endl;

    // pick it up
    my_arm->pick_up_and_delivery(arm_pos, end_pos);

    MsgSpeakOut("enjoy.wav");

    // go default
    ros::Duration(1.0).sleep();  // Sleep for 0.5 second
    my_arm->reset_arm_pos(default_pos);


}

/**
 * @brief Regenerate the voice by the given text message
 * 
 * @param text 
 * @param state 0: generate message online, 1: message already exist, broadcast it
 */
void arm_manager::MsgSpeakOut(const char* text) {

    printf("\n###########################################################################\n");
    
    std::string absolute_address = "/home/rover/collabrative_robot_ws/src/summer_research/voice-based-UI/xfei_asr/audios/";
    std::string combined = absolute_address + text;

    int n = combined.length();

    // declaring character array
    char char_array[n + 1];

    // copying the contents of the
    // string to char array
    strcpy(char_array, combined.c_str());

    play_wav(char_array);
    printf("Mplayer Run Success\n");
    printf("\n###########################################################################\n");
}
