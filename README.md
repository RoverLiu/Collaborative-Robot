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
# clone our repository
cd src

mkdir summer_research

cd summer_research

git clone https://github.com/RoverLiu/Collaborative-Robot.git


