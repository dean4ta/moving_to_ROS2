# moving_to_ROS2 #

## Getting Started ##

Building the container

```shell
make build # building ros1_bridge takes time and may be commented out in dockerfile

# run container with display
make up-display # make up-display-gpu with nvidia gpu and drivers installed
# run container without display
make up
```

Building and Sourcing the workspace

It is very likely that things will be weirdly broken inside the container on boot so follow these tips:


```shell
# if you want to build your workspace (exclude ros1_bridge)
source /opt/ros/${ROS2_DISTRO}/setup.bash
# Build workspace
colcon build --symlink-install --packages-skip ros1_bridge --cmake-args ' -DCMAKE_BUILD_TYPE=Release'
    

# if you need to build ros1_bridge:
source /opt/ros/${ROS1_DISTRO}/setup.bash
source /opt/ros/${ROS2_DISTRO}/setup.bash
MAKEFLAGS=-j37 # this is for good luck
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

```

Running

```shell
# terminal 1
source install/set up.bash
ros2 run ros1_bridge dynamic_bridge
# terminal 2
source install/setup.bash
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
```
