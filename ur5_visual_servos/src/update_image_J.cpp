#include <ros/ros.h>
#include "Jacobian_updater.cpp"
#include "image_capturer.cpp"
#include "tool_detector.cpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "update_image_J");
    ros::NodeHandle nh;

    // image topics
    std::string img_topic1 = "/visual_servo/camera1/image_raw_1";
    std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);
    visual_servo::ToolDetector detector(nh, std::vector<int>{20, 100, 100, 30, 255, 255});

    std::string J_topic = "/visual_servo/image_Jacobian";
    visual_servo::JacobianUpdater J_updater(nh, J_topic);

    J_updater.mainLoop(cam1, cam2, detector);

    ros::shutdown();
    return 0;
}

