# Panda Robotic Arm Following Demo Using ROS

## Overview
This repository is a catkin workspace created to be run on a real (or simulated) Franka Emika Panda
robotic arm using ROS. \
Main Demo: The end_effector_control package contains an interface that controls the panda end
effector to follow points given on a topic. The camera package contains code to retrieve and
publish such points using April tags and an Intel Real Sense depth cam. When run consecutively, the
Panda end effector will follow a linked April tag around its workspace.\
Environment Test: The environment testing demo samples a pre-made environment URDF file to test if
certain end effector poses are possible to reach while avoiding collisions.\
The ROS packages in the src folder are linked submodules that are also curated on this github
account. This inital version is built on Ubuntu 18.04 using ROS Melodic and the updated version is
built on Ubuntu 20.04 using ROS Noetic.

## Installation
**Note: These instructions assume use of zshell, if you are not using zshell then replace `.zsh` with whatever shell you are using (ex: bash -> `.bash`)**
### Melodic Version Installation (Ubuntu 18.04)
#### Versions
  * Refer to compatibility [chart](https://frankaemika.github.io/docs/compatibility.html)
  * Linux OS: Ubuntu `18.04`
  * ROS: `Melodic`
  * Panda system: `v3.0.0`
  * Libfranka: `0.7.1` (`robo-demo-melodic` branch)
    - Compatable Libfranka and Panda system versions: https://frankaemika.github.io/docs/libfranka_changelog.html
  * MoveIt: `1`
  * Franka Ros: `melodic-devel` branch
  * Python: Mix of `v2` (for moveit) and `v3` (camera)

#### Install ROS Melodic (for Ubuntu 18.04)
  * Instructions on ROS Wiki: https://wiki.ros.org/melodic/Installation/Ubuntu

#### Download and build `libfranka` from source
  1. Uninstall existing installations to avoid conflicts \
  `sudo apt remove "*libfranka*"`
  1. Install dependencies \
  `sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`
  1. Download source code (downloaded in home directory here) \
  `cd ~` \
  `git clone --recursive git@github.com:dwya222/libfranka.git`
  1. Checkout correct version and update \
  `cd ~/libfranka`
  `git checkout robo-demo-melodic` \
  `git submodule update`
  1. Build
  `mkdir build` \
  `cd build` \
  `cmake -DCMAKE_BUILD_TYPE=Release ..` \
  `cmake --build .`

#### Download and build `robo_demo_ws` from source
  1. Download source code (downloaded in home directory here) \
  `cd ~`
  `git clone --recursive git@github.com:dwya222/robo_demo_ws.git`
  1. Checkout demo version \
  `cd ~/robo_demo_ws` \
  `git checkout robo-demo-melodic`
  1. Initialize and clone submodules \
  `cd ~/robo_demo_ws` \
  `git submodule update --init --recursive`
  1. Clean the workspace if a different version has been built previously \
  `catkin clean`
  1. Install dependencies and build `robo_demo_ws` using the `libfranka` repo \
  `cd ~/robo_demo_ws` \
  `rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka` \
  `catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build`
  1. Source the workspace \
  `source ~/robo_demo_ws/devel/setup.zsh` \
  If you want to source every time you open a terminal: \
  `echo “source ~/robo_demo_ws/devel/setup.zsh” >> .zshrc`

### Noetic Version Installation (Ubuntu 20.04)
#### Versions
  * Refer to compatibility [chart](https://frankaemika.github.io/docs/compatibility.html)
  * Linux OS: Ubuntu `20.04`
  * ROS: `Noetic`
  * Panda system: `v4.0.4`
  * Libfranka: `0.8.0` (`robo-demo-melodic` branch)
    - Compatable Libfranka and Panda system versions: https://frankaemika.github.io/docs/libfranka_changelog.html
  * MoveIt: `1`
  * Franka Ros: `noetic-devel` branch
  * Python: `3`

#### Install ROS Noetic (for Ubuntu 20.04)
  * Instructions on ROS Wiki: https://wiki.ros.org/noetic/Installation/Ubuntu

#### Download and build `libfranka` from source
  1. Uninstall existing installations to avoid conflicts \
  `sudo apt remove "*libfranka*"`
  1. Install dependencies \
  `sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`
  1. Download source code (downloaded in home directory here) \
  `cd ~` \
  `git clone --recursive git@github.com:dwya222/libfranka.git`
  1. Checkout correct version and update \
  `cd ~/libfranka`
  `git checkout robo-demo-noetic` \
  `git submodule update`
  1. Build
  `mkdir build` \
  `cd build` \
  `cmake -DCMAKE_BUILD_TYPE=Release ..` \
  `cmake --build .`

#### Download and build `robo_demo_ws` from source
  1. Download source code (downloaded in home directory here) \
  `cd ~`
  `git clone --recursive git@github.com:dwya222/robo_demo_ws.git`
  1. Checkout demo version \
  `cd ~/robo_demo_ws` \
  `git checkout robo-demo-noetic`
  1. Initialize and clone submodules \
  `git submodule update --init --recursive`
  1. Clean the workspace if a different version has been built previously \
  `catkin clean`
  1. Specify Franka build path directory in environment variable \
  `export Franka_DIR=~/libfranka/build`
  1. Install dependencies and build `robo_demo_ws` using the `libfranka` repo \
  `cd ~/robo_demo_ws` \
  `rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka` \
  `catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build`
  1. Source the workspace \
  `source ~/robo_demo_ws/devel/setup.zsh` \
  If you want to source every time you open a terminal: \
  `echo “source ~/robo_demo_ws/devel/setup.zsh” >> .zshrc`

## Running the Code
### Run the demo in simulation
  1. `roslaunch end_effector_control demo.launch setup:=staged`
  1. `rosrun end_effector_control follow_point_subscriber.py`
  1. `rosrun camera main.py`
### Calibrate and Run the demo on the real panda bot
  1. Prerequisites:
     - Intel RealSense camera positioned facing the Panda arm ~1.5 meters away so that it has
       a good view of the robot workspace.
     - April Tag attached to the Panda end effector as in the staged_panda.urdf file.
     - Small box ~(6cm x 6cm x 6cm) with April tag attached to one side
     - Robot in program-op mode (blue light)
  1. Start the robot, wait till lights are yellow solid
  1. Use the browser to  navigate to the Franka control panel
     - `172.16.0.2/desk/`
     - firefox may raise security concerns, ok them
     - Unlock the joints (TODO: image of what we should see?)
     - Should see the yellow light on Panda turn white
  1. Hold the trigger (black with gray button) before running the next code
      - Should see the white light turn blue
      - This step cannot be done later, failing to do this step prevents robot from moving
  1. Run the following ROS launch command to start the demo calibration: \
     `roslaunch end_effector_control calibrate_demo.launch`
      - The robot will move through 28 poses with the end effector facing forward so that the
        camera can view the April Tag attached to it. Once the robot stops moving after the 28th
        pose, the calibration is completed and the process can be terminated (ctrl-c).
      - NOTE: this step only needs to be completed once after moving the camera to a new position
      - NOTE: to successfully complete the calibration the camera should be placed ~1.5 meters in
        front of the robot arm so that it can view all of the calibration poses.
  1. Run the following ROS launch command to start the demo: \
     `roslaunch end_effector_control run_demo.launch`
      - You should see the block appear in RViz, followed by a movement simulation, followed by
        actual movement. The RRT may get stuck if the block is not reachable.

### Running the environment test
1. Run the simulation for a premade position URDF (positions 1-7 are currently added) \
`roslaunch end_effector_control demo.launch setup:=bmi_p#` \
ex: for position 4, use arg `setup:=bmi_p4` \
*URDF files located at `end_effector_control/config/bmi_positions`*
2. `rosrun end_effector_control environment_test.py`
3. Run the `end_effector_control/scripts/test_results/df_to_png.py` file to get the final results .png images for each test \
`python3 ~/robo_demo_ws/src/end_effector_control/scripts/test_results/df_to_png.py`
