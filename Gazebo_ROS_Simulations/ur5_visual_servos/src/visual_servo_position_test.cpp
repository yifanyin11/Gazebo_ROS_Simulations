#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/RobotTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "visual_servo_controller.cpp"
#include "image_capturer.cpp"
#include "tool_detector.cpp"

int main(int argc, char** argv){
    // Ros setups
    ros::init(argc, argv, "visual_servo_position_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // MOVEIT planning setups
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setMaxVelocityScalingFactor(0.02);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.01);
    bool success;

    // get current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    // define target pose and initialize it as current pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position = current_pose.pose.position;

    ros::Rate rate(10);
    // target topics
    std::string t_topic = "/visual_servo/targets";

    // image topics
    std::string img_topic1 = "/visual_servo/camera1/image_raw_1";
    std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);
    visual_servo::ToolDetector detector_target(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_tool(nh, std::vector<int>{150, 150, 150, 160, 255, 255});

    double tol = 3.0;

    visual_servo::VisualServoController servo_controller(nh, t_topic, tol);
    std::cout << "Done initialize servo controller" << std::endl;

    Eigen::VectorXd increment;
    while(nh.ok()&&(!servo_controller.stopSign())){
        servo_controller.directionIncrement(increment, cam1, cam2, detector_tool);
        std::cout << "Done increment" << std::endl;
        target_pose.position.x = target_pose.position.x+increment(0);
        target_pose.position.y = target_pose.position.y+increment(1);
        target_pose.position.z = target_pose.position.z+increment(2);
        move_group_interface_arm.setPoseTarget(target_pose);
        move_group_interface_arm.move();
    }

}
