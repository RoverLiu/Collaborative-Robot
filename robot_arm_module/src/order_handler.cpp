/**
 * @file order_handler.cpp
 * @author Rover
 * @brief Handles the order from UI
 * Order is taken from /chocolate2pick topic under voice_command message type.
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "order_handler.h"
#include <iostream>
#include <string> // for string class
#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include "collabriative_bot_msgs/voice_command.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include <list>

/**
 * @brief Construct a new order handler::order handler object
 * 
 * @param nh ROS node
 * @param nh_priv Previous ROS node
 */
order_handler::order_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv) : _nh(nh), _nh_priv(nh_priv) 
{
    order_sub = nh.subscribe("chocolate2pick", 1000, & order_handler::newOrderCallback, this);
}

/**
 * @brief Destroy the order handler::order handler object
 * 
 */
order_handler::~order_handler() {}

/**
 * @brief print out all data saved
 * 
 */
void order_handler::print_data() {
    std::cout<<"Here is the order saved: "<<std::endl;
    for (int i : order_to_pick) {
        std::cout<<i<<std::endl;
    }
}

/**
 * @brief publish the order from saved vector (first come, first out/serve)
 * 
 * @return int The id for the chocolate
 */
int order_handler::get_chocolate_to_pick() {
    int choc;
    if (order_to_pick.empty()) 
    {
        return -1;
    }
    choc = order_to_pick.back();
    order_to_pick.pop_back();
    return choc;
}

/**
 * @brief callback when a new order is published in the topic
 * 
 * @param msg 
 */
void order_handler::newOrderCallback(const collabriative_bot_msgs::voice_command::ConstPtr& msg) 
{
    order_to_pick.push_front(msg->chocolate_type);
}