# Collaborative-Robot
This is a yumi based summer project.

# ####################################

# Environment set up
## make a directory
mkdir -p collabrative_robot_ws/src

cd collabrative_robot_ws

catkin_make

# Package from others
Packages below are required to be cloned into workspace.
## Camera package
The camera model is intel D435i. This type of camera has its support for linux - [realsence SDK package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and ROS - [realsence-ros](https://github.com/IntelRealSense/realsense-ros). Also, the [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure.git) package is required to run the ROS interface.

The librealsense coulld be tested by:
>realsense-viewer

To check the correct installation of ROS wrapper, here are two launch file to run:
```
1.roslaunch realsense2_camera rs_rgbd.launch 

2. roslaunch realsense2_camera rs_camera.launch initial_reset:=true 
```

## Object 3D estimation package
In this project, we use the [find-object](http://wiki.ros.org/find_object_2d) package to detect and localize the objects. Here is the installation steps:
```
1. install package 2d image detection: sudo apt-get install ros-melodic-find-object-2d
2. source file : https://github.com/introlab/find-object
3. 3. Run launch file: roslaunch camera_module 3D_position_estimation.launch 
```

Here is another package we did not apply but is quite useful for color detection. It is a package for [Color based 2D Localization](https://github.com/dovanhuong/object_detection_2d)

## Robot arm control
In this project, the abb yumi is applied as the robot arm. To control the yumi robot, several packages are applied as mentioned below:
```
1. install industrial_core: https://github.com/ros-industrial/industrial_core

2. install hector-xacro-tools: sudo apt install ros-melodic-hector-xacro-tools

3. get urdf file for yumi: https://github.com/OrebroUniversity/yumi.git

4. Get Yumi Control File from my github (capable for ros-kinetic, 16.04 and ros-melodic, 18.04): https://github.com/RoverLiu/yumi.git

5. Intall move it: sudo apt install ros-melodic-moveit
```

I have already included the moveit configuration in my package, but you can create a robot arm with moveit with instructions below:
https://www.youtube.com/watch?v=l4dtSRvlAjg

## voice package
This is one way to do the voice recognition. It is the [Modified ROS wrapper for pocketsphinx](https://github.com/UTNuclearRoboticsPublic/pocketsphinx/tree/melodic-devel) To reduce the complexity of the whole program, this package is not applied in this projecct. I do make a package for voice user interface based on [iflytex](https://www.iflytek.com/index.html). 

In general, you have to replace all APPID, resource file and library file in that package.
I am working on a different repo for UI. Check: https://github.com/RoverLiu/voice-based-UI.git
More details would be discussed at that package.

# Starting the RAPID scripts (IMPORTANT) 
This action is required to configure yumi hardware. All steps below should be applied by the yumi control pannel.

Everytime you power on the YuMi, or you close the YuMi ROS nodes running on your PC, you need to run the RAPID scripts again (they are Normal tasks inside the IRC5 controller due to safety reasons).

To do that, press the following physical buttons on the FlexPendant:
```

    Toggle the Motors On on the FlexPendant by pressing the upper-right corner button that has three horizontal lines.

    Toggle the Auto mode on the FlexPendant by pressing the upper-right corner button that has two horizontal lines.

    Move all the program pointers of the RAPID routines to main(), by pressing the upper-right corner button that has one centered horizontal line.

    Press the Play button located in the bottom-right corner.
```
You should see and hear the grippers performing three movements: a full opening, a full closing, and then opening again up to 10cm. This is the grippers calibration sequence, and it is advisable to wait for it to finish before running the YuMi ROS nodes.


## Our repository
In this repo, I have built an interface to manage the robot arm, voice UI, and camera module. Details could be found in the header files.

Here are steps to clone out repo:
```
cd src

mkdir summer_research

cd summer_research

git clone https://github.com/RoverLiu/Collaborative-Robot.git
```

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
We de have python version issue when you run under kinetics. Here is the way to hack:
sudo update-alternatives --config python3


