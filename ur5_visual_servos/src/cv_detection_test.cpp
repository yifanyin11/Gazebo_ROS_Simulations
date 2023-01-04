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
    std::string img_topic = "/visual_servo/camera1/image_raw_1";

    // detection setups
    visual_servo::ImageCapturer cam(nh, img_topic);
    visual_servo::ToolDetector detector(nh, img_topic, cam);

    while(nh.ok()){
        detector.detect(cam);
        detector.drawDetectRes();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}