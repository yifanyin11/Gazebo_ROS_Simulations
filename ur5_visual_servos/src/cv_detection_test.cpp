#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "image_capturer.cpp"
#include "tool_detector.cpp"

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "cv_detection_test");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    // image topics
    std::string img_topic = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam(nh, img_topic);
    visual_servo::ToolDetector detector(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_tip(nh, std::vector<int>{20, 100, 100, 30, 255, 255});

    while(nh.ok()){
        detector.detect(cam);
        detector.drawDetectRes();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}