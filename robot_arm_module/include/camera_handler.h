/**
 * @file camera_handler.h
 * @author Rover
 * @brief handles the data published by the camer module
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAMERA_HANDLER_H
#define __CAMERA_HANDLER_H

#include <iostream>
#include <string> // for string class
#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <string>
#include <unordered_map>

/**
 * @brief this class handles the results published from camera module
 * 
 */

class camera_handler
{
    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        ros::Subscriber object_sub;

        // save the data for object positions
        // the sequence is : middle_x, middle_y, width, height
        std::unordered_map<int, std::vector<float>> object_data;

        // call back
        void objectsDetectedCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    public:
        // data
        // specific id for each object
        // value for vertical_chocolate.bin
        const int kitkat_nestle = 53;
        const int kitkat_mint = 52;
        const int kitkat_gold = 61;
        const int kitkat_cookie_collision = 63;
        const int gripper_left = 55;
        const int gripper_right = 57;

        // value for clear_view.bin
        // const int kitkat_chuncky = 28;
        // const int kitkat_mint = 30;
        // const int kitkat_gold = 29;
        // const int snickers = 26;
        // const int gripper_left = 44;
        // const int gripper_right = 36;
        
        // methods
        camera_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~camera_handler();

        /**
         * @brief print out the object position details received
         * 
         */
        void print_data();

        /**
         * @brief Get the pos of object
         * 
         * @param id The chocolate ID 
         * @return std::vector<float> The position detail of the chocolate
         * Order: x,y,width,height
         */
        std::vector<float> get_pos(int id);
};



#endif