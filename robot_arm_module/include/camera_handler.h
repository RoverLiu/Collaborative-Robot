#ifndef __ARM_MANAGER_H
#define __ARM_MANAGER_H

#include <iostream>
#include <string> // for string class
#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief this class handles the results published from camera module
 * 
 */

class camera_handler {
    private:
        /* data */
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;



    public:
        camera_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~camera_handler();


};



#endif