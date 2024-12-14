# hero_chassis_controller

## Overview

The Hero Chassis Controller is a comprehensive ROS package tailored for Mecanum-wheeled robot chassis, featuring advanced PID control, odometry calculation, and inverse kinematics for velocity transformations. This package supports real-time parameter tuning and the ability to toggle between global and local coordinate systems for versatile operational scenarios. Additionally, the package integrates seamlessly with tools like PlotJuggler and rqt_reconfigure, making it easier to visualize and fine-tune performance metrics. The updated keyboard control node offers intuitive command input for controlling the robot.

**Keywords:** Mecanum wheel, PID control, odometry, velocity transformation,keyboard_control_node

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Richard<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Richard, wang13530080747@gmail.co**m

The `hero_chassis_controller` package has been tested under ROS Noetic on Ubuntu 20.04.

It is designed for educational and research purposes and may be updated frequently.



## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- control_toolbox (PID control library)
- dynamic_reconfigure (for runtime parameter tuning)
- tf (transformations)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd ~/catkin_ws/src
	git clone https://github.com/Richard-cc172/hero_chassis_controller.git
	cd ~/catkin_ws
	catkin_make

### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. Make sure
to [install Docker](https://docs.docker.com/get-docker/) first.

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash

This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`)
, gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/Richard-cc172/hero_chassis_controller.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch hero_chassis_controller hero_chassis_controller.launch

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch hero_chassis_controller pid_controller.launch 
	
	roslaunch hero_chassis_controller velocity_transform.launch 

Control the robot using:

```
roslaunch hero_chassis_controller keyboard_control_node.launch 
```

To switch between local and global coordinate systems, modify the `speed_mode` parameter in the YAML configuration file and restart the node.



## Config files

* **chassis_params.yaml** 

  Contains the following configurable parameters:

  - **`wheel_radius`**: Radius of the wheels.
  - **`wheel_base`**: Distance between the front and rear wheels.
  - **`wheel_track`**: Distance between the left and right wheels.



* **chassis_controller.yaml** 

  Contains the following configurable parameters for the chassis controller:

  - `joints`**:** Specifies the names of the wheel joints controlled by the node:
    - `left_front_wheel_joint`
    - `right_front_wheel_joint`
    - `left_back_wheel_joint`
    - `right_back_wheel_joint`
  - `pid`**:** Defines the PID parameters for the controller:
    - `p`: Proportional gain (default: `2.0`).
    - `i`: Integral gain (default: `0.1`).
    - `d`: Derivative gain (default: `0.01`).
    - `i_clamp`: Limits the integral windup (default: `1.0`).
  - `joint_state_controller`**:** Publishes the joint states of the robot:
    - `type`: Type of the controller (default: `joint_state_controller/JointStateController`).
    - `publish_rate`: Frequency of state publishing (default: `50 Hz`).
  
  
  
  

- ##### speed_mode.yaml

​	Contains the following configurable parameter for speed mode:

​	speed_mode: Specifies the coordinate system for velocity control:

​		`	local`: Interprets velocity commands in the robot’s local coordinate system.

​		`global` Interprets velocity commands in the global coordinate system.

##### 

## Launch files

* ##### velocity_transform.launch

  Configures and launches the velocity transformation node:

  - **Parameters Loaded:**
  - `chassis_params.yaml`: Defines the wheel and chassis dimensions.
    - `speed_mode.yaml`: Specifies the coordinate system for velocity control.

* **pid_controller.launch**

  Sets up the PID controller for managing wheel velocities:

  - **Parameters Loaded:**
    - `chassis_controller.yaml`: Configures PID parameters for each wheel.
    - `chassis_params.yaml`: Provides chassis dimensions.
  
  

- ##### keyboard_control_node.launch

  Launches the keyboard control node for manual teleoperation:

  - **Parameters:**
    - `linear_speed`: Specifies the robot's linear speed (default: `0.5`).
    - `angular_speed`: Specifies the robot's angular speed (default: `0.5`).



## Nodes

### hero_chassis_controller_node

Handles PID-based control for Mecanum wheels and computes odometry.

- **Subscribed Topics:**
  - `/transformed_cmd_vel`: ([geometry_msgs/Twist]) Velocity commands for each wheel.
  - `/joint_states`: ([sensor_msgs/JointState]) Wheel speed feedback.
- **Published Topics:**
  - `/odom`: ([nav_msgs/Odometry]) Odometry data for the robot.
- **Parameters:**
  - `wheel_base`: Distance between the front and back wheels.
  - `wheel_track`: Distance between the left and right wheels.
  - `wheel_radius`: Radius of the wheels.



### velocity_transform_node

Transforms velocity commands between global and local coordinate systems.

- **Subscribed Topics:**
  - `/cmd_vel`: ([geometry_msgs/Twist]) Input velocity commands.
- **Published Topics:**
  - `/transformed_cmd_vel`: ([geometry_msgs/Twist]) Transformed velocity commands in the appropriate coordinate system.
  - `/joint_states`: ([sensor_msgs/JointState]) Wheel speed commands.
- **Parameters:**
  - `speed_mode`: Switch between `local` and `global` modes.



### keyboard_control_node

Handles manual control inputs from the keyboard.

- **Published Topics:**
  - `/cmd_vel`: ([geometry_msgs/Twist]) Linear and angular velocity commands based on user input.
- **Parameters:**
  - `linear_speed`: Configurable linear speed for forward/backward motion (default: `0.5`).
  - `angular_speed`: Configurable angular speed for rotation (default: `0.5`).



## Testing

Use the following commands for testing:

- Publish velocity commands:

  ```
  rostopic pub /cmd_vel geometry_msgs/Twist -r 10 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
  ```

- Visualize odometry in RViz and verify the robot's behavior.

### Tools

- Use `rqt_reconfigure` to tune PID parameters in real-time.

- Use `plotjuggler` to visualize the robot's odometry and wheel velocities.

  

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
