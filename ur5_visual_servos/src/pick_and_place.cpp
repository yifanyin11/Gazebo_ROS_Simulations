#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


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

  // // Visualization setups
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("world");
  // visual_tools.deleteAllMarkers();
  // visual_tools.loadRemoteControl();
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // Print all available groups in the robot
  ROS_INFO_NAMED("pick_and_place", "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
          move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  
  // 1. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
  
  bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  std::cout << "Done homing" << std::endl;

  // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
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

  // ROS_INFO_NAMED("pick_and_place", "Visualizing plan as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose, "pose");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan_arm.trajectory_, joint_model_group);
  // visual_tools.trigger();

  move_group_interface_arm.move();

  std::cout << "Done placing the TCP" << std::endl;

  // 3. Open the gripper
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_gripper.move();

  std::cout << "Done Opening the gripper" << std::endl;

  // 4. Move the TCP close to the object
  target_pose.position.z = target_pose.position.z - 0.15;
  move_group_interface_arm.setPoseTarget(target_pose);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  std::cout << "Done moving the TCP close to the object" << std::endl;

  // 5. Close the  gripper
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_gripper.move();

  std::cout << "Done closing the gripper" << std::endl;

  // 6. Move the TCP above the plate
  target_pose.position.z = target_pose.position.z + 0.15;
  target_pose.position.x = target_pose.position.x - 0.6;
  move_group_interface_arm.setPoseTarget(target_pose);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  std::cout << "Done moving the TCP above the plate" << std::endl;

  // 7. Lower the TCP above the plate
  target_pose.position.z = target_pose.position.z - 0.08;
  move_group_interface_arm.setPoseTarget(target_pose);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  std::cout << "Done lowering the TCP above the plate" << std::endl;

  // 8. Open the gripper
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_gripper.move();

  std::cout << "Done opening the gripper" << std::endl;

  // 9. Back to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
  
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  std::cout << "Done homing" << std::endl;

  ros::shutdown();
  return 0;
}
