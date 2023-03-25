# Robotics Algorithm Implementations

# cas726
---
This package contains the following modules:
- occpancy grid mapping
- landmark-based mapping

> Note: Code is for educational purposes and is not optimized for performance

#### Using Landmark-based mapping

1. Start the simulation environment

    ```
    export IGN_GAZEBO_RESOURCE_PATH=install/cas726/share/cas726/worlds/
    ros2 launch cas726 tugbot_bringup.launch.py
    ```
    
2. Start the obj_extractor node: `ros2 run obj_extractor obj_extractor`.
3. Start the landmark mapping node `ros2 run cas726 landmark_mapper`.

# obj_extractor
---
This package contains an object detection module.

You can test this module by running

```
ros2 run obj_extractor obj_extractor
ros2 run obj_extractor test_image_sub

# Then run any node that streams images to the `/front/color_image` topic.
```

# cas726_interfaces
---
This contains message and service definitions.

# Developing
---
Using [this](https://hub.docker.com/repository/docker/tstoyanov/ros2/general) prebuilt docker container. The following commands are required before running the launch files:





