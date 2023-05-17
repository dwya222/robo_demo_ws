# Panda Robotic Arm Following Demos Using ROS

## Table of Contents
1. [Repo Overview](#repo-overview)
    * [Block Following Demo Overview](#block-following-demo-overview)
    * [Servo Position Control Demo Overview](#servo-position-control-demo-overview)
    * [Parallel Real-Time RRT\* (PRT-RRT\*) Demo Overview](#parallel-real-time-rrt-prt-rrt-demo-overview)
2. [Install the Demos](#install-the-demos)
3. [Run the Demos](#run-the-demos)
    * [Run the Block Following Demo](#run-the-block-following-demo)
    * [Run the Servo Position Control Demo](#run-the-servo-position-control-demo)
    * [Run the (PRT-RRT\*) Demo](#run-the-prt-rrt-demo)

## Repo Overview
This repository is a catkin workspace created to run demos for a real or simulated Franka Emika
Panda robotic arm using ROS. The ROS packages in the src folder are linked submodules that are also
curated on this github account. The most recent version is built on Ubuntu 20.04 using ROS Noetic.
There are 3 demos: the block following demo, the servo position control demo, and the Parallel
Real-Time RRT* (PRT-RRT*) demo.
### Block Following Demo Overview
Once calibrated, a RealSense depth camera takes readings of the position of an April Tag,
transforms them to the Panda base link frame, and publishes them live to a topic. Another node
subscribes to these points and uses MoveIt to plan and executes a path to place the end effector
directly above the April Tag position, ready to grasp whatever object the April Tag is attached to.
### Servo Position Control Demo Overview
A RealSense depth camera takes readings of the position of an April Tag and publishes them live to
a topic. A servo control node reads these positions and maps the position of the panda end effector
to the position of the April Tag. Initially when the position of the April Tag is read, that
position will be mapped to the start position of the Panda end effector. Once the camera detects a
displacement of the April Tag from the start position, the Panda end effector will be controlled to
mimic that displacement.
### Parallel Real-Time RRT\* (PRT-RRT\*) Demo Overview
**TODO**
### Test-ing (parens)


## Install the Demos
**Note: These instructions assume use of zshell, if you are not using zshell then replace `.zsh`
with whatever shell you are using (ex: bash -> `.bash`)**

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

## Run the Demos
### Run the Block Following Demo
#### Run the demo in simulation with camera hardware in the loop
  1. `roslaunch end_effector_control demo.launch setup:=staged`
  1. `rosrun end_effector_control follow_point_subscriber.py`
  1. `rosrun camera main.py`
#### Calibrate and run the demo on the real panda bot
  1. Prerequisites:
     - Intel RealSense camera positioned facing the Panda arm ~1.5 meters away so that it has
       a good view of the robot workspace.
     - April Tag attached to the Panda end effector as in the staged_panda.urdf file.
     - Small box ~(6cm x 6cm x 6cm) with April tag attached to one side
     - Panda Robot in program-op mode (blue light)
  1. Start the robot, wait till lights are yellow solid
  1. Use the browser to  navigate to the Franka control panel
     - `172.16.0.2/desk/`
     - firefox may raise security concerns, ok them
     - Unlock the joints
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
### Run the Servo Position Control Demo
  1. Prerequisites:
     - Intel RealSense camera
     - Small box ~(6cm x 6cm x 6cm) with April tag attached to one side
     - Panda Robot in program-op mode (blue light)
  1. Start the robot, wait till lights are yellow solid
  1. Use the browser to  navigate to the Franka control panel
     - `172.16.0.2/desk/`
     - firefox may raise security concerns, ok them
     - Unlock the joints
     - Should see the yellow light on Panda turn white
  1. Hold the trigger (black with gray button) before running the next code
      - Should see the white light turn blue
      - This step cannot be done later, failing to do this step prevents robot from moving
  1. Run the following ROS launch command to start the servoing position control demo: \
     `roslaunch end_effector_control servo_position_tracking_demo.launch`
      - You should see the arm move to the start position (if not already there), and then the end
        effector will be controlled to mimic any motion of the small box with the April tag
        attached.
      - NOTE: The April tag on the small box must be in view of the camera, the demo will not begin
        until it can see the April tag, and the robot will be controlled to the last commanded
        position and stop if the April tag goes out of view.
      - NOTE: The demo will use the first position of the box as seen by the camera as the neutral
        point, and any deviation from the neutral point will be mimicked by the end effector.
### Run the PRT-RRT\* Demo
**TODO**
