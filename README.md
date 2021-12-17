# Collaborative-Robot
This is a yumi based summer project.

# ####################################

# Environment set up
## make a directory
mkdir -p collabrative_robot_ws/src

cd collabrative_robot_ws

catkin_make

# Package from others
## voice package
https://github.com/UTNuclearRoboticsPublic/pocketsphinx/tree/melodic-devel

## realsence SDK package
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

after intallation, Run realsense-viewer

## realsence-ros package
https://github.com/IntelRealSense/realsense-ros

## Run RGBD camera 
realsense-viewer
On Rivz: roslaunch realsense2_camera rs_rgbd.launch 

## Object 3D estimation 
1. install package 2d image detection: sudo apt-get install ros-melodic-find-object-2d
2. source file : https://github.com/introlab/find-object
3. launch file from: https://github.com/bandasaikrishna/object_detection_and_3d_pose_estimation

## Color based 2D Localization:
https://github.com/dovanhuong/object_detection_2d

## Robot arm control
### install industrial_core
https://github.com/ros-industrial/industrial_core

### get urdf file for yumi
https://github.com/OrebroUniversity/yumi.git

### Yumi Control File (ros-kinetic, 16.04)
https://github.com/kth-ros-pkg/ridgeback_yumi.git

### move it
sudo apt install ros-melodic-moveit

### create a robot arm with moveit
https://www.youtube.com/watch?v=l4dtSRvlAjg

# clone our repository
cd src

mkdir summer_research

cd summer_research

git clone https://github.com/RoverLiu/Collaborative-Robot.git


