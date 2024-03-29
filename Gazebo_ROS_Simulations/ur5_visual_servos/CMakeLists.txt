cmake_minimum_required(VERSION 3.0.2)
project(ur5_visual_servos)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools

  cv_bridge
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  control_msgs

  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs 
  sensor_msgs
  trajectory_msgs 
  control_msgs
  geometry_msgs
  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES
    image_capturer
    tool_detector
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  SYSTEM 
  include
  src/lm
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(image_capturer src/image_capturer.cpp)
target_link_libraries(image_capturer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(tool_detector src/tool_detector.cpp)
target_link_libraries(tool_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(Jacobian_updater src/Jacobian_updater.cpp)
target_link_libraries(Jacobian_updater ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(gradient_updater src/gradient_updater.cpp)
target_link_libraries(gradient_updater ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(visual_servo_controller src/visual_servo_controller.cpp)
target_link_libraries(visual_servo_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_library(pixel_picker src/pixel_picker.cpp)
target_link_libraries(pixel_picker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur5_visual_servos.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(set_home_position src/set_home_position.cpp)
target_link_libraries(set_home_position ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(robot_homing src/robot_homing.cpp)
target_link_libraries(robot_homing ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(async_move_test src/async_move_test.cpp)
target_link_libraries(async_move_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cv_detection_test src/cv_detection_test.cpp)
target_link_libraries(cv_detection_test pixel_picker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(update_image_J src/update_image_J.cpp)
target_link_libraries(update_image_J ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(visual_servo_position_test src/visual_servo_position_test.cpp)
target_link_libraries(visual_servo_position_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(visual_servo_full_pose_test src/visual_servo_full_pose_test.cpp)
target_link_libraries(visual_servo_full_pose_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(update_ori_gradient src/update_ori_gradient.cpp)
target_link_libraries(update_ori_gradient ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(visual_servo_orientation_test src/visual_servo_orientation_test.cpp)
target_link_libraries(visual_servo_orientation_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(target_publisher src/target_publisher.cpp)
target_link_libraries(target_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
