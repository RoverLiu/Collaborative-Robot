#ifndef __ORDER_HANDLER_H
#define __ORDER_HANDLER_H

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
 * @brief this class handles the results published from voice module
 * 
 */

class order_handler
{
    private:
        /* data */
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        ros::Subscriber order_sub;

        // save the detail about order to pick
        std::list<int> order_to_pick;

        // call back
        void newOrderCallback(const collabriative_bot_msgs::voice_command::ConstPtr& msg);


    public:
        order_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~order_handler();

        void print_data();

        // return the id for the chocolate
        // return -1 when there is no chocolate to pick
        int get_chocolate_to_pick();


};



#endif