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

order_handler::order_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv) : _nh(nh), _nh_priv(nh_priv) 
{
    order_sub = nh.subscribe("chocolate2pick", 1000, & order_handler::newOrderCallback, this);
}

order_handler::~order_handler() {}

void order_handler::print_data() {
    std::cout<<"Here is the order saved: "<<std::endl;
    for (int i : order_to_pick) {
        std::cout<<i<<std::endl;
    }
}

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

void order_handler::newOrderCallback(const collabriative_bot_msgs::voice_command::ConstPtr& msg) 
{
    order_to_pick.push_front(msg->chocolate_type);
}