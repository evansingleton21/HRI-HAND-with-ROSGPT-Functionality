This repository contains the two ROS Packages I used and developed during my Bachelor of Mechatronics Engineering Final Year Dissertation Project.

To run the ROSGPT Functionality, there is a ROSLAUNCH file in hri_hand_control/launch titled hri_hand_control.launch
and a ROSLAUNCH file in rosgpt/launch titled rosgpt.launch that both need to be launched

Prerequisites:
ROS1 Noetic
Gazebo Classic
hri_hand_control package
rosgpt package
Personal API key

API key needs to be saved as an environment variable or parsed into the model-calling section of the rosgpt node.

This repo will be updated with independent joint control once I have the free-time to do so and refine it.

A future adaptation into ROS2 syntax and capability will be in development.

Acknowledgements:
The HRI-Hand used in this project is from https://github.com/hj26-park/HRI-Hand-ROS
ROSGPT basic fundamentals are from: 
https://github.com/aniskoubaa/rosgpt (ROS2) 
https://www.linkedin.com/pulse/rosgpt-beginners-simple-guide-integrating-chatgpt-ros-del-valle-mnhfe/ (ROS1)

This dissertation project was supervised by Dr. Sunny Katyara whom provided the code skeletal backbone, and ROS guidance 
that I didn't have when starting out. 
