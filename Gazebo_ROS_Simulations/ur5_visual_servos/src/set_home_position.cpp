// This script read the current robot state into a config file as the home position for homing process
/* Author: Yifan Yin */

#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "set_home_position");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(100);

    static const std::string PLANNING_GROUP = "ur5_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // edit ros params
    for (int i=0;i<joint_group_positions.size();++i){
        std::string name = "/home_position/"+joint_names[i];
        ros::param::set(name, joint_group_positions[i]);
        nh.setParam(name, joint_group_positions[i]);
    }

    // write to the configuration file
    std::string path = ros::package::getPath("ur5_visual_servos");
    YAML::Node config = YAML::LoadFile(path+"/config/config.yaml");
    for (int i=0;i<joint_group_positions.size();++i){
        std::string name = joint_names[i];
        config["home_position"][name] = joint_group_positions[i];
    }
    std::ofstream fout(path+"/config/config.yaml");
    ROS_INFO("Writing home position into configuration file ...");
    fout << config;
    return 0;
}