# ROS1 Portfolio
![Static Badge](https://img.shields.io/badge/ROS-Melodic-green)
![Static Badge](https://img.shields.io/badge/Platform-AgileX_LIMO-blue)
![Static Badge](https://img.shields.io/badge/OS-Ubuntu%2018.04-blue)

## Overview
In this project, teams are assigned to design an arena and explore the mapping and navgation capabilities of the AgileX LIMO which is based on ROS (Robot Operating System). Extensive documentation on the LIMO can be found on their [GitHub repository](https://github.com/agilexrobotics/limo-doc/).

## Key Aspects
- Uses ROS Melodic (Ubuntu 18.04)
- ARM-based hardware (Nvidia Jetson Nano)
- LiDAR and depth vision-based sensing (YDLIDAR XL2 & Orbbec Dabai)
- RViz visualization

## Why ROS?
- Open-source software frameworks: Provides users with the flexibility when choosing tools and libraries for use with their application, all while being free and supported by a large community.
- Large user base: Widespread use from universities to industries as a result of its open-source nature.

## Mapping of arena
- For this project, we have decided to use `rtabmap` mapping algorithm, which can take advantage of the RGB-D data from the Orbbec Dabai depth camera and the EAI X2L 2D LiDAR to map out the arena.

## Real-time object detection
-  Using `darknet_ros` and Orbbec Dabai camera, the LIMO can detect and identify objects that are in the camera view.
-  In addition to common objects such as persons and traffic lights, uncommon objects like information signs can also be trained locally and then trigger a routine which will tell the LIMO what to do.
-  Although this was ultimately not implemented due to time constraints and hardware limitations (storage, RAM), it was proposed as a method for navigating through our arena where the obstacles are not walled high enough for the rtabmap to detect.
