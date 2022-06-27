# quad_ros

I haven't had time to write the documentation yet. But I tried my best to keep the whole testing process as simple as possible. And did not change the code of ocs2 itself.

## Build

- ocs2 is a huge monorepo, you just need to compile ocs2_legged_robot_ros and its dependencies!
- If you only run simulation, you do not need to compile unitree_hw of quad_ros;

## Quick Start

1. Set your robot type(a1 or aliengo) as an environment variable: ROBOT_TYPE
```
export ROBOT_TYPE=a1
```

2. Run the simulation:
```
roslaunch unitree_description empty_world.launch
```
3. Load the controller:
```
mon launch quad_controllers load_controller.launch
```  
4. You can start the controller using `rqt_controller_manager`
```
sudo apt install ros-noetic-rqt-controller-manager
rosrun rqt_controller_manager rqt_controller_manager
```
5. After start the controller, you can command the robot by two terminal created by step 3.
