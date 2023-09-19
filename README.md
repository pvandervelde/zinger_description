# zinger_description

Provides the geometric description of the zinger robot for use with
[RViz](http://wiki.ros.org/rviz), [Gazebo](https://gazebosim.org/) and other [ROS 2](https://docs.ros.org/en/humble/index.html#)
nodes.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
robot using Raspberry Pi OS.

1. [ROS humble](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) with the
   `robot_state_publisher`, the `joint_state_broadcaster` and the
   [ros_control](https://control.ros.org/master/index.html) packages.
1. A working [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

## Contents

This repository contains different folders for the different parts of the robot description.

* The config files that provide the configurations for the ROS control actuators and the test publishers
  * [config/zinger.yaml](config/zinger.yaml) defines the settings for the position controller, used
    for the steering angle of each wheel, and the velocity controller, used to drive the wheel forwards
    or backwards.
* The launch directory contains the launch files
  * [launch/base.launch.py](launch/base.launch.py) - Launches the `robot_state_publisher` which is
    always needed.
  * [launch/controller_manager.launch.py](launch/controller_manager.launch.py - Launches the ROS control
    `controller_manager` node. Should not be launched if the robot is being simulated in Gazebo.
  * [launch/controllers.launch.py](launch/controllers.launch.py) - Launches the appropriate ROS controllers
    for the robot.
  * [launch/robot_description.launch.py](launch/robot_description.launch.py) - The launch file for
    the robot description. Will launch all the required nodes.
  * [launch/test_joint_trajectory_controller.launch.py](launch/test_joint_trajectory_controller.launch.py) - Launches
    the test publisher for the steering angle changes.
  * [launch/test_velocity_controller.launch.py](launch/test_velocity_controller.launch.py) - Launches the test
    publisher for the wheel rotation changes.
* The [rviz](rviz/) directory contains the configuration file for RViz
* The URDF files that describe the robot for ROS, RViz and Gazebo.
  * [urdf/base.xacro](urdf/base.xacro) - The geometric description of the robot with the different
    [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint)
    elements.
  * [urdf/gazebo.xacro](urdf/gazebo.xacro) - The Gazebo specific additions to the model, e.g. the
    materials for the different links and the ROS control integration.
  * [urdf/macros.xacro](urdf/macros.xacro) - The XACRO macros for the robot. Contains the macros for
    the steering modules that consist of a wheel and a wheel bracket.
  * [urdf/materials.xacro](urdf/materials.xacro) - Specifies the material properties for Gazebo and Rviz

## Usage

In general the `zinger_description` package will not directly be launched. It is designed to be
referenced by other SCUTTLE ROS packages.
