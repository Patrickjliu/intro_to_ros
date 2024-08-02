
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
~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:169.254.90.79:14550
```

### Launching MAVROS

Connect to the SITL:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 gcs_url:=udp://:14550@169.254.71.248:14550 tgt_system:=1 tgt_component:=0 system_id:=255 component_id:=0
```

### Running Node

Run an individual node:

```bash
ros2 run intro_to_ros your_executable_name
```

### Rosmav Setup
https://prod.liveshare.vsengsaas.visualstudio.com/join?E1D25FBEB53EDF5028190DAC395A07F4212D
```
cd ~/auvc_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select rosmav --symlink-install

source ~/auvc_ws/install/setup.zsh

```

```
~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:169.254.166.217:14550
```

```
cd ~/auvc_ws

ros2 run rosmav ros_bluerov2_interface
```

```
cd ~/auvc_ws

ros2 run rosmav bluerov2_camera_interface
```

```
cd ~/auvc_ws

ros2 run rosmav arm
```

```
cd ~/auvc_ws

ros2 run rosmav depth_control

ros2 topic pub /bluerov2/desired_depth std_msgs/msg/Float64 "{data: 5}"
```




### Dt-Apriltags Note
```auvc_ws pip install dt-apriltags       


error: externally-managed-environment

× This environment is externally managed
╰─> To install Python packages system-wide, try apt install
    python3-xyz, where xyz is the package you are trying to
    install.
    
    If you wish to install a non-Debian-packaged Python package,
    create a virtual environment using python3 -m venv path/to/venv.
    Then use path/to/venv/bin/python and path/to/venv/bin/pip. Make
    sure you have python3-full installed.
    
    If you wish to install a non-Debian packaged Python application,
    it may be easiest to use pipx install xyz, which will manage a
    virtual environment for you. Make sure you have pipx installed.
    
    See /usr/share/doc/python3.12/README.venv for more information.

note: If you believe this is a mistake, please contact your Python installation or OS distribution provider. You can override this, at the risk of breaking your Python installation or OS, by passing --break-system-packages.
hint: See PEP 668 for the detailed specification.
➜  auvc_ws pip install dt-apriltags --break-system-packages


Defaulting to user installation because normal site-packages is not writeable
WARNING: Skipping /usr/lib/python3.12/dist-packages/argcomplete-3.1.4.dist-info due to invalid metadata entry 'name'
Collecting dt-apriltags
  Using cached dt_apriltags-3.1.7-py3-none-manylinux2014_aarch64.whl.metadata (924 bytes)
Requirement already satisfied: numpy in /usr/lib/python3/dist-packages (from dt-apriltags) (1.26.4)
Using cached dt_apriltags-3.1.7-py3-none-manylinux2014_aarch64.whl (1.2 MB)
WARNING: Skipping /usr/lib/python3.12/dist-packages/argcomplete-3.1.4.dist-info due to invalid metadata entry 'name'
Installing collected packages: dt-apriltags
Successfully installed dt-apriltags-3.1.7
➜  auvc_ws```