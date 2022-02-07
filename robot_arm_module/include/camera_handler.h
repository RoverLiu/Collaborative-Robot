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
        /* data */
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        ros::Subscriber object_sub;

        void objectsDetectedCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);



    public:
        camera_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~camera_handler();

        // data
        // specific name for each object
        const int kitkat_chuncky = 15;
        const int kitkat_mint = 16;
        const int kitkat_gold = 17;
        const int snickers = 18;


};



#endif