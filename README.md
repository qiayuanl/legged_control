# legged_control

I haven't had time to write the documentation yet. But I tried my best to keep the whole testing process as simple as
possible. And did not change the code of ocs2 itself.

## Build

- ocs2 is a huge monorepo, you just need to compile ocs2_legged_robot_ros and its dependencies!
- If you only run simulation, you do not need to compile unitree_hw;

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
mon launch legged_controllers load_controller.launch
```

4. You can start the controller using `rqt_controller_manager`

```
sudo apt install ros-noetic-rqt-controller-manager
rosrun rqt_controller_manager rqt_controller_manager
```

5. After start the `legged_controller` or `legged_cheater_controller`, **NOTE that you are not allowed to start the `legged_cheater_controller` in real hardware!**

6. Set the gait then use RViz and other tools to control the robot
![ezgif-5-684a1e1e23.gif](https://s2.loli.net/2022/07/27/lBzdeRa1gmvwx9C.gif)
