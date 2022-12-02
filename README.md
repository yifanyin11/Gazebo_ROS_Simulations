# Gazebo_ROS_Simulations
This ROS package is built for implementing and testing applications of robot arms related to robot manipulations, motion planning and vision-based control. A simulation environment has been constructed using Gazebo, provided rich contents to simulate related experimental setups, such as depth cameras, Robotiq grippers, AR markers, force sensors as well as workbenches, targets and reference objects.

![alt text](https://github.com/yifanyin11/Gazebo_ROS_Simulations/blob/main/repo_data/example.jpg?raw=true)

## Tasks
Although the simulation environment is designed for all variety of tasks of manipulators, there are several tasks in the package "ur5_visual_servos" that can serve as examples:

1. Robot homing process
2. Simple pick and place task
3. 3D point-target visual servoing

## Getting Started

### Dependencies

The package runs on Ubuntu 20.04 and ROS noetic.

### Installing

The packages will have to be built from sources in a Catkin workspace:
```
cd $HOME/catkin_ws/src

# retrieve the sources
git clone https://github.com/yifanyin11/Gazebo_ROS_Simulations.git

# install dependencies from sources
git clone https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/filesmuggler/robotiq.git

# checking other dependencies
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y

# building
catkin build

# activate this workspace
source devel/setup.bash
```

### Executing program

* Launch the simulated worlds
```
roslaunch ur5_gripper_moveit_config visual_servoing_world.launch
```
Gazebo GUI should be launched. You may start the simulation by clicking the "play" button on the bottom toolbar. Rviz interface will show up right after and sychonize the pose of the robot in the simulation.

* Robot homing process

There are two ways to perform robot homing. A simple way is to use the Motion Planning Rviz plugin. Follow the steps below:


```
code blocks for commands
```

## Description

An in-depth paragraph about your project and overview of use.


## Authors

Contributors names and contact info

Yifan Yin

Email: [yyin34@jhu.edu](yyin34@jhu.edu)

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* []()
* []()
* []()
