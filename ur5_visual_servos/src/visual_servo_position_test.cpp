#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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
    move_group_interface_arm.setMaxVelocityScalingFactor(0.05);
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
    // image topics
    std::string img_topic1 = "/visual_servo/camera1/image_raw_1";
    std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);
    visual_servo::ToolDetector detector_target(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_tool(nh, std::vector<int>{20, 100, 100, 30, 255, 255});

    std::cout << "Done setups" << std::endl;

    cv::Point target1, target2;
    detector_target.detect(cam1);
    std::cout << "Done detect target1" << std::endl;
    target1 = detector_target.getCenter();
    detector_target.drawDetectRes();
    std::cout << "Done assign target1" << std::endl;
    detector_target.detect(cam2);
    std::cout << "Done detect target2" << std::endl;
    target2 = detector_target.getCenter();
    detector_target.drawDetectRes();
    std::cout << "Done assign target2" << std::endl;

    std::cout << "Done detect targets" << std::endl;
    
    int num_features = 4;

    Eigen::VectorXd targets;
    targets.resize(num_features);
    targets << target1.x, target1.y, target2.x, target2.y;
    std::cout << "Done initialize targets" << std::endl;

    double tol = 10.0;
    visual_servo::VisualServoController servo_controller(nh, tol, targets);
    std::cout << "Done initialize servo controller" << std::endl;

    Eigen::VectorXd increment;
    while(nh.ok()&&(!servo_controller.stopSign())){
        servo_controller.directionIncrement(increment, cam1, cam2, detector_tool);
        std::cout << "Done increment" << std::endl;
        target_pose.position.x = target_pose.position.x+increment(0);
        target_pose.position.y = target_pose.position.y+increment(1);
        target_pose.position.z = target_pose.position.z+increment(2);
        move_group_interface_arm.setPoseTarget(target_pose);

        success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("visual_servo_position_test", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
        // move_group_interface_arm.asyncMove();
        move_group_interface_arm.move();
        // rate.sleep();
    }

}
