# Robotics Algorithm Implementations

## cas726

This package contains the following modules:
- occpancy grid mapping
- lankmard-based mapping

> Note: Code is for educational purposes and is not optimized for performance

## obj_extractor

This package contains an object detection module.

## cas726_interfaces

This contains message and service definitions.

# Developing

Using [this](https://hub.docker.com/repository/docker/tstoyanov/ros2/general) prebuilt docker container. The following commands are required before running the launch files:

```
ros2 launch cas726 tugbot_bringup.launch.py
export IGN_GAZEBO_RESOURCE_PATH=install/cas726/share/cas726/worlds/
```

