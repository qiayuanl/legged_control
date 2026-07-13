# Isaac Sim 5.1 integration (Go1)

This branch connects the ROS 1 `legged_control` stack to a Go1 running in
Isaac Sim through a small TCP bridge. It was developed with Isaac Sim 5.1,
ROS 2 Humble on the host, and ROS 1 Noetic in Docker.

## Data flow

```text
Isaac Sim + go1_isaac_bridge.py (ROS 2)
       |  joint state, IMU, odometry, clock
       v
ros2_isaac_socket_bridge.py (host TCP server, port 50055)
       |  newline-delimited JSON over TCP
       v
ros1_socket_bridge.py (ROS 1 container)
       |  /isaac/* topics
       v
IsaacHW -> controller_manager -> NMPC/WBC
       |  12 x [q_des, dq_des, kp, kd, feed-forward]
       +-------------------------------------------------> Isaac Sim
```

The TCP link is intentionally local and unauthenticated. Bind it only to a
trusted host/network and do not expose port 50055 to the Internet.

## Included files

- `isaac_legged_hw/`: catkin hardware-interface package, ROS 1 TCP client,
  planned-contact publisher, optional velocity-command filter, and launch files.
- `isaac_sim/scripts/go1_isaac_bridge.py`: run inside Isaac Sim after opening
  the stage. It applies torque and publishes simulated sensors on ROS 2.
- `isaac_sim/scripts/ros2_isaac_socket_bridge.py`: ROS 2 TCP server on the host.
- `isaac_sim/start_isaac_legged_stack.sh`: starts the host bridge and the ROS 1
  stack in Docker without opening machine-specific terminal windows.
- `isaac_sim/assets/go1_comp.usd`: tested stage override that composes the
  NVIDIA Go1 asset. See the asset README for the dependency/licensing note.

## Prerequisites

1. A working upstream `legged_control` catkin workspace and the OCS2
   dependencies listed in the main README.
2. ROS 1 Noetic in the controller environment (a Docker container is fine).
3. Isaac Sim 5.1 with its ROS 2 Humble bridge installed and enabled.
4. The NVIDIA Go1 asset available to Isaac Sim.
5. Host/container TCP connectivity. On Linux, create the container with
   `--add-host=host.docker.internal:host-gateway`, or pass the Docker bridge
   gateway explicitly as `socket_host:=172.17.0.1`.

The controller and Isaac Sim must run on the same little-endian machine; this
setup was tested locally, not across an untrusted or high-latency network.

## Build the ROS 1 side

Clone this branch into the catkin workspace's `src` directory, then build:

```bash
cd ~/legged_ws
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build isaac_legged_hw legged_controllers legged_unitree_description
source devel/setup.bash
```

No file under `build/`, `devel/`, `logs/`, any bag file, or any local Isaac
cache belongs in Git.

## Run

The safe startup order is controller first, Isaac timeline last. Starting the
timeline before the controller is running leaves the robot at zero torque and
it will fall until control becomes active.

### Recommended: portable stack launcher

After building the branch in the container workspace, run this on the host:

```bash
./isaac_sim/start_isaac_legged_stack.sh
```

Its defaults match the development setup. Override them when needed:

```bash
CONTAINER_NAME=my_container \
ROS1_WS=/root/legged_ws \
ROS2_SOCKET_HOST=host.docker.internal \
./isaac_sim/start_isaac_legged_stack.sh
```

The launcher starts the container, host ROS 2 socket server, `roscore`, Isaac
hardware bringup, loads the cheater controller, and switches it to `running`.
It intentionally does not start Isaac or publish velocity. Continue with step
1 below while leaving the Isaac timeline stopped, run the bridge script, and
only then press **Play**. The remaining manual steps document the same services
for users who do not use the launcher.

Use separate terminals so failures remain visible.

### 1. Start Isaac Sim

Activate the Isaac Sim Python environment, then:

```bash
./isaac_sim/run_isaac_sim.sh
```

Open `isaac_sim/assets/go1_comp.usd` while the timeline is stopped. Open Isaac
Sim's Script Editor, load `isaac_sim/scripts/go1_isaac_bridge.py`, and run it.
After the controller is `running`, press **Play**. The startup
summary must report these prims:

```text
Robot root prim: /World/go1
Articulation root: /World/go1/trunk
```

If your stage uses other paths, edit the configuration block at the top of
`go1_isaac_bridge.py` before running it.

### 2. Start the ROS 2 socket server on the host

Source only ROS 2 in this terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
python3 isaac_sim/scripts/ros2_isaac_socket_bridge.py
```

`ISAAC_SOCKET_BIND` and `ISAAC_SOCKET_PORT` override the default bind address
(`0.0.0.0`) and port (`50055`).

### 3. Start ROS 1 hardware and bridges in the container

Start `roscore`, then in a sourced catkin terminal run:

```bash
export ROBOT_TYPE=go1
roslaunch isaac_legged_hw bringup_isaac.launch \
  socket_host:=host.docker.internal
```

If the container was not created with the `host.docker.internal` mapping, use:

```bash
roslaunch isaac_legged_hw bringup_isaac.launch socket_host:=172.17.0.1
```

The launch file starts `IsaacHW`, the ROS 1 TCP client, and the MPC-mode to
contact-state publisher. Its defaults are a 500 Hz hardware loop, 0.002 s
cycle error threshold, and real-time priority 95.

### 4. Load and start the controller

In another sourced ROS 1 terminal:

```bash
roslaunch isaac_legged_hw control_isaac.launch cheater:=true
```

After the controller has loaded:

```bash
rosservice call /controller_manager/switch_controller \
  "start_controllers: ['controllers/legged_cheater_controller']
stop_controllers: []
strictness: 2
start_asap: false
timeout: 0.0"
```

The cheater controller consumes `/ground_truth/state`; do not use it on real
hardware. This launch also selects the Isaac-specific neutral-abduction posture
from `isaac_legged_hw/config/go1_reference.info` and enables the persistent
velocity-reference converter. The ordinary upstream controller launch keeps
the original Go1 posture and target converter.

### 5. Send a command

First select a gait from an interactive ROS 1 container terminal:

```bash
rosrun ocs2_legged_robot_ros legged_robot_gait_command
```

For example, select `trot`, then publish a conservative command. Begin at
`0.05` m/s and increase it only after checking stable motion:

Direct command, matching upstream behavior:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Stop the publisher and send an explicit zero command before ending the test.

For the optional path/yaw stabilizer, start bringup with
`start_cmd_filter:=true` and publish commands to `/cmd_vel_user` instead. A
fresh zero command means stop; when commands time out, the filter publishes
nothing. Setting `linear.z` above `0.5` requests the filter's hold mode.

## Interface contract

| Direction | Topic | Message | Notes |
|---|---|---|---|
| Isaac to ROS 1 | `/isaac/joint_states` | `sensor_msgs/JointState` | 12 named joints |
| Isaac to ROS 1 | `/isaac/imu` | `sensor_msgs/Imu` | angular velocity in body frame |
| Isaac to ROS 1 | `/isaac/odom` | `nav_msgs/Odometry` | twist in world frame |
| ROS 1 local | `/isaac/contacts` | `std_msgs/Float64MultiArray` | order LF, LH, RF, RH |
| ROS 1 to Isaac | `/isaac/joint_cmd` | `std_msgs/Float64MultiArray` | 60 values; five per joint |
| ROS 1 public | `/ground_truth/state` | `nav_msgs/Odometry` | used by cheater estimation |

Joint order is LF, LH, RF, RH, with HAA, HFE, KFE for each leg. Each joint
command is `[q_des, dq_des, kp, kd, feed_forward]`. The Isaac script converts
that hybrid command to torque, clips it to its configured limit, and applies
it with `ArticulationAction`.

Contacts deliberately come from the MPC mode schedule. Isaac's geometric foot
contact estimate is published only as `/isaac/contacts_debug`; the ROS 1 TCP
client drops it to prevent two publishers from fighting over `/isaac/contacts`.

## Checks and troubleshooting

The complete GUI path was exercised with Isaac Sim 5.1: all 12 joints were
mapped, the controller remained `running`, ROS 1 joint state arrived at about
231 Hz, and a 3-second `0.05` m/s trot moved the base about 0.12 m forward. The
robot returned to a stable, upright stance at approximately 0.30 m base height
after the zero command. Rates depend on rendering and host load.

Run these before enabling motion:

```bash
rostopic hz /isaac/joint_states
rostopic hz /isaac/imu
rostopic hz /isaac/odom
rostopic echo -n 1 /isaac/contacts
rostopic echo -n 1 /clock
rostopic list | grep -E 'isaac|ground_truth|controller_manager'
```

- No TCP connection: verify the host address from inside the container with
  `getent hosts host.docker.internal` and ensure port 50055 is not firewalled.
- No Isaac topics: confirm Simulation is playing, the ROS 2 bridge is enabled,
  and the Isaac script completed without a missing-prim error.
- No `/clock`: the ROS 1 bridge derives it from the Isaac odometry header to
  avoid competing clock publishers.
- Robot receives no torque: confirm `/isaac/joint_cmd` contains 60 values and
  the controller is in `running` state using `controller_manager/list_controllers`.
- Wrong or unstable motion: stop the controller first. Verify the USD prim and
  joint names, spawn height, joint order, and physics frequency before tuning
  gains or torque limits.

## Safety

Start with no velocity command and keep the robot clear of obstacles. This is
research software: the bridge clips values but is not a certified safety
controller. Never connect the cheater controller or these simulation torque
settings directly to physical hardware.
