// This script read the home position from a config file and home the robot to that position
/* Author: Yifan Yin */
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "robot_homing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(1000);

    // moveit setups for planning
    static const std::string PLANNING_GROUP = "ur5_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::vector<double> joint_group_positions;
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

    // get ros params
    for (int i=0;i<joint_group_positions.size();++i){
        std::string name = "/home_position/"+joint_names[i];
        if (nh.getParam(name, joint_group_positions[i]));
        else{
            ROS_ERROR("No parameter named %s!", name);
        }
    }

    // set joint space goal
    move_group.setJointValueTarget(joint_group_positions);

    // plan 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_homing", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    //move the robot
    move_group.move();

    ros::shutdown();
    return 0;
}