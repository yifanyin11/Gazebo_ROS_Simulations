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

This ROS package runs on Ubuntu 20.04 and ROS noetic.

### Installing

The package will have to be built from sources in a Catkin workspace:
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

### Executing programs

* Launch the simulated worlds
```
roslaunch ur5_gripper_moveit_config visual_servoing_world.launch
```
Gazebo GUI should be launched. You may start the simulation by clicking the "play" button on the bottom toolbar. Rviz interface will show up right after and synchronize the pose of the robot in the simulation.

* Robot homing process

There are two ways to perform robot homing. A simple way is to send the robot to the existing home configuration defined in the .srdf. Follow the steps below:

1. Go to the MotionPlanning panel in Rviz, select "Planning" tab.
2. Change the Planning Group from the default value of "gripper" to "ur5_arm" using the dropdown menu.
3. Leave the Start State as <current>; For the Goal State of the planning, select "home" at the bottom of the dropdown. 
4. Select "Plan" under Commands. Check the trajectory, "Execute" if it's as expected.
  
As the other option, you may also customize the home configuration. To do that, you need to move the end effector around to find your desired home pose and execute the planning. Then, set the robot current pose as home configuration by running
```
rosrun ur5_visual_servos set_home_position
```
It will record the joint states of that pose in the project configuration file for any later reference. Once done, you may home the robot at any pose by simply calling
```
rosrun ur5_visual_servos robot_homing
```

* Simple pick and place task

A blue cube and a plate are placed on the workbrench. A simple pick and place task can be performed by running

```
rosrun ur5_visual_servos pick_and_place
```

## Authors

Contributors names and contact info

Yifan Yin

Email: [yyin34@jhu.edu](yyin34@jhu.edu)

## Acknowledgments

Inspiration, code snippets, etc.
* [Simulation of a Robotiq gripper](https://github.com/filesmuggler/robotiq)
