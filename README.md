# Collaborative-Robot
This is a yumi based summer project.

# ####################################

# Environment set up
## make a directory
mkdir -p collabrative_robot_ws/src
cd collabrative_robot_ws
catkin_make

# Package from others

# clone our repository
cd src
git clone https://github.com/RoverLiu/Collaborative-Robot.git

# ##################################

# run the simulation
## go to correct directory
cd pizzaBot_ws
source devel/setup.bash

## build the workspace
catkin_make

## run kitchen simulator
roslaunch kitchen_simulator kitchen.launch

## run user interface
roslaunch user_interface user_interface.launch 

## gazebo window

### if the model is not loaded into the gazebo, run this instruction in the terminal
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/<<your directory>>/pizzaBot_ws/src/MTRX3760-Major-Project/pizza_town/models

### choose buger as the model
export TURTLEBOT3_MODEL=burger

### run gazebo 
roslaunch pizza_town pizza_town_2_bot.launch

## run rviz
roslaunch pizza_town multiple_bot.rviz.launch

## run april tag detecction
source Apriltag_detection/devel/setup.bash
roslaunch apriltag_ros two_robot.launch

## Now you can put order into from UI
