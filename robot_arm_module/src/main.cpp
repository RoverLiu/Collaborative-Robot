#include "arm_manager.h"
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char* argv[]) {
    // Initialize the ros
    ros::init( argc, argv, "arm_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );

    arm_manager my_manager(nh, nh_priv);

    
}