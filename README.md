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

## ddynamic_reconfigure
https://github.com/pal-robotics/ddynamic_reconfigure.git

## Run RGBD camera 
realsense-viewer

On Rivz: 

1.roslaunch realsense2_camera rs_rgbd.launch (Not Working - error without message)

2. roslaunch realsense2_camera rs_camera.launch initial_reset:=true (could see depth data)


## Object 3D estimation 
1. install package 2d image detection: sudo apt-get install ros-melodic-find-object-2d
2. source file : https://github.com/introlab/find-object
3. launch file from: https://github.com/bandasaikrishna/object_detection_and_3d_pose_estimation
4. Run launch file: roslaunch camera_module 3D_position_estimation.launch 

## Color based 2D Localization:
https://github.com/dovanhuong/object_detection_2d

## Robot arm control
### install industrial_core
https://github.com/ros-industrial/industrial_core

### install hector-xacro-tools
sudo apt install ros-melodic-hector-xacro-tools

### get urdf file for yumi
https://github.com/OrebroUniversity/yumi.git

### Yumi Control File (ros-kinetic, 16.04 / ros-melodic, 18.04)
https://github.com/RoverLiu/yumi.git

### move it
sudo apt install ros-melodic-moveit

### create a robot arm with moveit
https://www.youtube.com/watch?v=l4dtSRvlAjg



## Voice interface
https://blog.csdn.net/xiao9469/article/details/109716656
#### APPID:2433b578
### Run speech reconigition
1. roscore
2. rosrun xfei_asr  iat_publish_speak (publish and subscribe)
3. rostopic echo /xfspeech  (speech text generate)
4. rostopic echo /xfwords   (Wrong message will show up here)
5. rostopic pub xfwakeup std_msgs/String "ok"

We are working on a different repo for UI. Check: https://github.com/RoverLiu/voice-based-UI.git


# Starting the RAPID scripts (IMPORTANT)

Everytime you power on the YuMi, or you close the YuMi ROS nodes running on your PC, you need to run the RAPID scripts again (they are Normal tasks inside the IRC5 controller due to safety reasons).

To do that, press the following physical buttons on the FlexPendant:

    Toggle the Motors On on the FlexPendant by pressing the upper-right corner button that has three horizontal lines.

    Toggle the Auto mode on the FlexPendant by pressing the upper-right corner button that has two horizontal lines.

    Move all the program pointers of the RAPID routines to main(), by pressing the upper-right corner button that has one centered horizontal line.

    Press the Play button located in the bottom-right corner.

You should see and hear the grippers performing three movements: a full opening, a full closing, and then opening again up to 10cm. This is the grippers calibration sequence, and it is advisable to wait for it to finish before running the YuMi ROS nodes.


# clone our repository
cd src

mkdir summer_research

cd summer_research

git clone https://github.com/RoverLiu/Collaborative-Robot.git

#  Run
Please run all modules in the sequence below:

1. Go to yumi_depends_ws and source the setup.bash and run following two command:
    - roslaunch yumi_launch yumi_traj_pos_control.launch
    - roslaunch yumi_moveit_config move_group.launch
2. Go to collabrative_robot_ws and source the setup.bash
    - roslaunch camera_odule 3D_position_estimation.launch
    - rosrun robot_arm_module arm_manager
    - roslaunch xfei_asr UI.launch

# Modify python version
sudo update-alternatives --config python3


