# Panda Robotic Arm Following Demo Using ROS

## Overview
This repository is a catkin workspace created to be run on a real (or simulated) Franka Emika Panda robotic arm using ROS. \
Main Demo: The end_effector_control package contains an interface that controls the panda end effector to follow points given on a topic. The camera package contains code to retrieve and publish such points using April tags and an Intel Real Sense depth cam. When run consecutively, the Panda end effector will follow a linked April tag around its workspace.\
Environment Test: The environment testing demo samples a pre-made environment URDF file to test if certain end effector poses are possible to reach while avoiding collisions.\
The ROS packages in the src folder are linked submodules that are also curated on this github account. This inital version is built on Ubuntu 18.04 using ROS Melodic.

## Installation
**Note: These instructions assume use of zshell, if you are not using zshell then replace `.zsh` with whatever shell you are using (ex: bash -> `.bash`)**
### Versions
  * Linux OS: Ubuntu `18.04`
  * ROS: `Melodic`
  * Panda system: `v3.0.0`
  * Libfranka: `0.7.1` (`robo-demo-melodic` branch)
    - Compatable Libfranka and Panda system versions: https://frankaemika.github.io/docs/libfranka_changelog.html
  * MoveIt: `1`
  * Franka Ros: `melodic-devel` branch

### Install ROS Melodic (for Ubuntu 18.04)
  * Instructions on ROS Wiki: https://wiki.ros.org/melodic/Installation/Ubuntu

### Download and build `libfranka` from source
  1. Uninstall existing installations to avoid conflicts \
  `sudo apt remove "*libfranka*"`
  2. Install dependencies \
  `sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`
  3. Download source code (downloaded in home directory here) \
  `cd ~` \
  `git clone --recursive git@github.com:dwya222/libfranka.git` \
  4. Checkout correct version and update \
  `cd ~/libfranka`
  `git checkout robo-demo-melodic` \
  `git submodule update`
  5. Build
  `mkdir build` \
  `cd build` \
  `cmake -DCMAKE_BUILD_TYPE=Release ..` \
  `cmake --build .`

### Download and build `robo_demo_ws` from source
  1. Download source code (downloaded in home directory here) \
  `cd ~` \
  `git clone --recursive git@github.com:dwya222/robo_demo_ws.git` \
  2. Initialize and clone submodules \
  `cd ~/robo_demo_ws` \
  `git submodule update --init --recursive` \
  3. Install dependencies and build `robo_demo_ws` using the `libfranka` repo \
  `cd ~/robo_demo_ws` \
  `rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka` \
  `catkin build -DCMAKE_BUILD_TYPE=Resease -DFranka_DIR:PATH=~/libfranka/build` \
  4. Source the workspace \
  `source ~/robo_demo_ws/devel/setup.zsh` \
  If you want to source every time you open a terminal: \
  `echo “source ~/robo_demo_ws/devel/setup.zsh” >> .zshrc`

## Running the Code
### Run the demo in simulation
  1. `roslaunch end_effector_control demo.launch setup:=staged`
  2. `roslaunch end_effector_control follow_point_subscriber.py`
  3. `roslaunch camera main.py`
### Run the demo on the real panda bot
  1. `roslaunch panda_moveit_config`
  2. `roslaunch end_effector_control follow_point_subscriber.py`
  3. `roslaunch camera main.py`
### Running the environment test
1. Run the simulation for a premade position URDF (positions 1-7 are currently added) \
`roslaunch end_effector_control demo.launch setup:=bmi_p#` \
ex: for position 4, use arg `setup:=bmi_p4` \
*URDF files located at `end_effector_control/config/bmi_positions`*
2. `rosrun end_effector_control environment_test.py`
3. Run the `end_effector_control/scripts/test_results/df_to_png.py` file to get the final results .png images for each test \
`python3 ~/robo_demo_ws/src/end_effector_control/scripts/test_results/df_to_png.py`
