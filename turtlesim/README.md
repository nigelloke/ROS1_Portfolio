# turtlesim

Using the included `turtlesim` in ROS, we can better understand nodes in ROS and much more. On top of that, we can interact with the virtual turtle by moving and spawning more turtles. For more information, you may refer to the ROS Wiki [here](http://wiki.ros.org/turtlesim/Tutorials)

## What does it do?
- Spawns a turtle that can be moved using topics and services
-  Ability to spawn more turtles during runtime
-  Draw shapes via publishing velocity commands

## What's needed?
- ROS (Melodic or Noetic) (To install, please follow the ROS Envrionment installation instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))
- turtlesim (if its not installed, you will need to install it with the following command)

  ```bash
  $ sudo apt-get install ros-melodic-turtlesim
  ```
  Replace `melodic` with `noetic` if your ROS installation is Noetic

## To get started

- Open a Terminal window, and run

  ```bash
  $ roscore
  ```
  
- Open another Terminal window, and run

  ```bash
  $ rosrun turtlesim turtlesim_node
  ```
  This will open the turtlesim

- Open another Terminal window, and run

  ```bash
  $ rosrun turtlesim turtle_teleop_key
  ```
  This will allow you to control the turtle with arrow keys

- Once all that's running, you should see this
  <img width="989" height="744" alt="image" src="https://github.com/user-attachments/assets/4d750337-6856-45f7-bd13-56f4cfa47da5" />

- Open another Terminal window, and run

  ```bash
  rosservice call /spawn 2.0 2.0 0.0 "turtle2"
  ```
  This will spawn another turtle named `turtle2` on the bottom left
  <img width="1172" height="748" alt="image" src="https://github.com/user-attachments/assets/c298f2d2-e963-4db7-a85d-3e2ea3a79210" />
