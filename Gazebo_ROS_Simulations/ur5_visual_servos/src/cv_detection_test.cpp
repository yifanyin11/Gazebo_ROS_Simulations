#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "image_capturer.cpp"
#include "tool_detector.cpp"
#include "pixel_picker.cpp"

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "cv_detection_test");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    // image topics
    std::string img_topic1 = "/visual_servo/camera1/image_raw_1";
    std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);
    visual_servo::ToolDetector detector_toolcenter(nh, std::vector<int>{150, 150, 150, 160, 255, 255});
    visual_servo::ToolDetector detector_tooltip(nh, std::vector<int>{20, 100, 100, 30, 255, 255});
    visual_servo::ToolDetector detector_frametip(nh, std::vector<int>{100, 150, 100, 110, 255,255});

    visual_servo::ToolDetector detector_target(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_target_tip1(nh, std::vector<int>{130, 30, 30, 140, 255, 120});
    visual_servo::ToolDetector detector_target_tip2(nh, std::vector<int>{76, 100, 150, 96, 255, 255});

    detector_toolcenter.detect(cam1);
    cv::Mat img = cam1.getCurrentImage();

    PixelPicker picker;

    cv::Point point = picker.pickOne(img);
    std::cout << point.x << " " << point.y << std::endl;

    // while(nh.ok()){
    //     // detector_target.detect(cam1);
    //     // detector_target.drawDetectRes();
    //     // detector_target.detect(cam2);
    //     // detector_target.drawDetectRes();

    //     // detector_target_tip1.detect(cam1);
    //     // detector_target_tip1.drawDetectRes();
    //     // detector_target_tip1.detect(cam2);
    //     // detector_target_tip1.drawDetectRes();

    //     // detector_target_tip2.detect(cam1);
    //     // detector_target_tip2.drawDetectRes();
    //     // detector_target_tip2.detect(cam2);
    //     // detector_target_tip2.drawDetectRes();

    //     detector_toolcenter.detect(cam1);
    //     detector_toolcenter.drawDetectRes();
    //     detector_toolcenter.detect(cam2);
    //     detector_toolcenter.drawDetectRes();

    //     detector_tooltip.detect(cam1);
    //     detector_tooltip.drawDetectRes();
    //     detector_tooltip.detect(cam2);
    //     detector_tooltip.drawDetectRes();

    //     detector_frametip.detect(cam1);
    //     detector_frametip.drawDetectRes();
    //     detector_frametip.detect(cam2);
    //     detector_frametip.drawDetectRes();
        
    //     rate.sleep();
    // }

    ros::shutdown();
    return 0;
}