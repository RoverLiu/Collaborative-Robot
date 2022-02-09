#include "arm_manager.h"
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "camera_handler.h"
#include "regression.h"
#include "order_handler.h"



int main(int argc, char* argv[]) {
    // Initialize the ros
    ros::init( argc, argv, "arm_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );

    ros::AsyncSpinner spinner(1);
    spinner.start();



    arm_manager my_manager(nh, nh_priv);
    // my_manager.wait();

    my_manager.calibration_new();

    while (true) 
    {
        ROS_INFO("enter while loop");
        my_manager.pick_up_chocolate();
    }

    // // open gripper
    // my_manager.left_arm->gripper_control(1);

    // // reset gripper direction
    // my_manager.left_arm->reset_griper_direction();

    // my_manager.wait();

    //-----------------------------------------------------------------------------------
    // // testing 
    // // move somewhere
    // geometry_msgs::Pose target_pose;
    // target_pose.orientation = my_manager.left_arm->get_direction(1);
    // target_pose.position.x = 0.20000;
    // target_pose.position.y = 0.20000;
    // target_pose.position.z = 0.20000;

    // my_manager.left_arm->auto_move_arm(target_pose);
    
    // my_manager.wait();

    // // trajectory
    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(target_pose);

    // geometry_msgs::Pose target_pose1 = target_pose;
    // //front
    // target_pose1.position.x += 0.30000;

    // std::cout<<target_pose1.position<<std::endl;
    // waypoints.push_back(target_pose1);  // down

    // target_pose1.position.y += 0.10000;
    // waypoints.push_back(target_pose1);  // right

    // // target_pose1.position.x += 0.40000;
    // // target_pose1.position.y += 0.40000;
    // // // target_pose1.position.x -= 0.20000;
    // // waypoints.push_back(target_pose1);  // up and left

    // my_manager.left_arm->CartesianPath_move_arm(waypoints);
    // my_manager.wait();

    // while (true) {
    //     my_manager.left_arm->get_current_pose();
    // }

    // test camera
    // camera_handler my_camera(nh, nh_priv);
    // while (true)
    // {
    //     my_camera.print_data();
    // }
    
    // test regression
    // std::vector<float> x = {1,2,3};
    // std::vector<float> y = {5,7,9};
    // regression reg(x,y);
    // // Printing the best fitting line
    // reg.PrintBestFittingLine();
    // std::cout << "Predicted value at 2060 = "
    //     << reg.predict(2060) << std::endl;
    // std::cout << "The errorSquared = "
    //     << reg.errorSquare() << std::endl;
    // std::cout << "Error in 2050 = "
    //     << reg.errorIn(2050) << std::endl;

    // pick up test
        // my_manager.wait();
    // my_manager.calibration();
    // geometry_msgs::Pose goal;
    // goal.position.x = 0.4000;
    // goal.position.y = 0.1000;
    // goal.position.z = 0.16000;

    // my_manager.left_arm->pick_up_and_delivery(goal);
    
    // order_handler test
    // order_handler my_handler(nh, nh_priv);
    // while(true) {my_handler.print_data();}

}