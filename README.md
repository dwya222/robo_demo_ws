# Panda Robotic Arm Following Demo Using ROS

This repository is a catkin workspace created to be run on a real (or simulated) Franka Emika Panda robotic arm using ROS. \
Main Demo: The end_effector_control package contains an interface that controls the panda end effector to follow points given on a topic. The camera package contains code to retrieve and publish such points using April tags and an Intel Real Sense depth cam. When run consecutively, the Panda end effector will follow a linked April tag around its workspace.\
The ROS packages in the src folder are linked submodules that are also curated on this github account. This inital version is build on Ubuntu 18.04 using ROS Melodic. 
