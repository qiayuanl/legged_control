# legged_control
## Introduction
`legged_control` is an NMPC-WBC legged robot control stack and framework based on [OCS2](https://github.com/leggedrobotics/ocs2) and [ros-control](http://wiki.ros.org/ros_control). To the author’s best knowledge, this framework is probably the best-performing open-source legged robot MPC control framework. Thanks to the ros-control interface, you can easily use this framework for your robot.

## Installation
### Source code
The source code is hosted on GitHub: [qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control).
```
# Clone legged_control
git clone git@github.com:qiayuanliao/legged_control.git
```

### OCS2
OCS2 is a huge monorepo; **DO NOT** try to compile the whole repo. You only need to compile `ocs2_legged_robot_ros` and its dependencies following the step below.

1. You are supposed to clone the OCS2, pinocchio, and hpp-fcl as described in the documentation of OCS2.
   ```
   # Clone OCS2
   git clone git@github.com:leggedrobotics/ocs2.git
   # Clone pinocchio
   git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
   # Clone hpp-fcl
   git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
   # Clone ocs2_robotic_assets
   git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
   # Install dependencies
   sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
   ```
2. Compile the `ocs2_legged_robot_ros` package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) instead of `catkin_make`. It will take you about ten minutes.
   ```
   catkin build ocs2_legged_robot_ros
   ```
   Ensure you can command the ANYmal as shown in the [document](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot) and below.
   ![](https://leggedrobotics.github.io/ocs2/_images/legged_robot.gif)
 
### Build
Build the source code of `legged_control` by:
```
catkin build legged_controllers
```

Build the simulation (**DO NOT** run on the onboard computer)
```
catkin build legged_gazebo
```
Build the hardware interface real robot. If you use your computer only for simulation, you **DO NOT** need to compile `unitree_hw` (TODO: add legged prefix to the package name)
```
catkin build unitree_hw
```

## Quick Start

1. Set your robot type as an environment variable: ROBOT_TYPE

```
export ROBOT_TYPE=a1
```

2. Run the simulation:

```
roslaunch unitree_description empty_world.launch
```
Or on the robot hardware:
```
roslaunch unitree_hw unitree_hw.launch
```

3. Load the controller:

```
roslaunch legged_controllers load_controller.launch cheater:=false
```

4. You can start the controller using `rqt_controller_manager`

```
sudo apt install ros-noetic-rqt-controller-manager
rosrun rqt_controller_manager rqt_controller_manager
```

5. Start the `legged_controller` or `legged_cheater_controller`, **NOTE that you are not allowed to start the `legged_cheater_controller` in real hardware!**

6. Set the gait in the terminal of `load_controller.launch`, then use RViz and other tools to control the robot

![ezgif-5-684a1e1e23.gif](https://s2.loli.net/2022/07/27/lBzdeRa1gmvwx9C.gif)

## Statistics

The table below shows lab successfully deploy this repo in their **real A1**

| Lab | Spend Time | 
| ---- | ---- | 
| XPeng Robotics | 1 day | 
| Unitree | - | 
| Geely Auto | 3 hours | 
| Hybrid Robotics | 2 hours |
