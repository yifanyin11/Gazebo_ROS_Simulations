// This script is the test of asynchonous move mode of MoveIt! interface
/* Author: Yifan Yin */
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    // ROS setups
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
    // Planning setups
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    // Print all available groups in the robot
    ROS_INFO_NAMED("pick_and_place", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    move_group_interface_arm.setMaxVelocityScalingFactor(0.03);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.1);
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    std::cout << "Done homing" << std::endl;

    // 2. Place the TCP to target 1 (above the blue box)
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    geometry_msgs::Pose target_pose;

    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.3;

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Start sleeping for 1.0 second ..." << std::endl;

    ros::Duration(1.0).sleep();

    std::cout << "Done sleeping; check TCP position" << std::endl;
    std::cout << "Ready to move to target 2" << std::endl;

    // 3. Move the TCP to target 2 (center of the box)
    target_pose.position.z = target_pose.position.z - 0.15;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Start sleeping for 0.5 second ..." << std::endl;

    ros::Duration(0.5).sleep();

    std::cout << "Done sleeping; check TCP position" << std::endl;
    std::cout << "Ready to move to target 3" << std::endl;

    // 4. Move the TCP to target 3 (raise it from the box)
    target_pose.position.z = target_pose.position.z + 0.3;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Start sleeping for 1.0 second ..." << std::endl;

    ros::Duration(1.0).sleep();

    std::cout << "Done sleeping; check TCP position" << std::endl;

    ros::shutdown();
    return 0;
}
