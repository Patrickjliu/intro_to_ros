
# BlueROV2 ROS2 Setup Guide

This guide provides the necessary commands to build a ROS2 package, source it, and run MAVROS and the simulator.

## ROS2 Terminal Commands

### Create a New Package

Execute this in a workspace's `src` directory (e.g., `~/auvc_ws/src`):

```bash
ros2 pkg create --build-type ament_python your_package_name
ros2 pkg create --build-type ament_python intro_to_ros
```



### Running Nodes and Topics

Run an individual node:

```bash
ros2 run your_package_name your_executable_name
ros2 run intro_to_ros your_executable_name
```

List all active nodes:

```bash
ros2 node list
```

List all named topics:

```bash
ros2 topic list
```

Check the message type of a topic:

```bash
ros2 topic type /your/topic
```

Echo messages on a specific topic:

```bash
ros2 topic echo /your/topic
```

Manually publish a message:

```bash
ros2 topic pub -N 10 /your_topic std_msgs/String '{data: "hello world"}'
```

Check the frequency of messages on a topic:

```bash
ros2 topic hz /your/topic
```

### Source ROS2

Source the ROS2 installation manually:

```bash
source /opt/ros/jazzy/setup.zsh
```

Source a workspace manually:

```bash
source ~/auvc_ws/install/setup.zsh
```

### Launching ArduSub

Start a SITL:

```bash
~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:169.254.64.190:14550
```

### Launching MAVROS

Connect to the SITL:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 gcs_url:=udp://:14550@169.254.64.190:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240
```

### Getting Your Computer's IP

To get your computer's IP address, use:

```bash
hostname -I
```

Now you're ready to build, source, and run your ROS2 projects with BlueROV2!





# Consise Setup

### Build Packages

Build all packages in a workspace (execute in the workspace directory, e.g., `~/auvc_ws`):

Create symbolic links while building:

```bash
colcon build --packages-select intro_to_ros --symlink-install
```

### Source ROS2

Source a workspace manually:

```bash
source ~/auvc_ws/install/setup.zsh
```

### Launching ArduSub

Start a SITL:

```bash
~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp::14550
```

### Launching MAVROS

Connect to the SITL:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 gcs_url:=udp://:14550@:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240
```

### Running Node

Run an individual node:

```bash
ros2 run intro_to_ros your_executable_name
```