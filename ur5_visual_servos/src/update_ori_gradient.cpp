#include <ros/ros.h>
#include "gradient_updater.cpp"
#include "image_capturer.cpp"
#include "tool_detector.cpp"


int main(int argc, char** argv){
    ros::init(argc, argv, "update_ori_gradient");
    ros::NodeHandle nh;
    
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
    visual_servo::ToolDetector detector_target_frametip(nh, std::vector<int>{130, 30, 30, 140, 255, 120});
    visual_servo::ToolDetector detector_target_tooltip(nh, std::vector<int>{76, 100, 150, 96, 255, 255});

    std::cout << "Done setups" << std::endl;

    int num_features = 4;

    cv::Point target1, target2;
    cv::Point target_tooltipPos1, target_tooltipPos2;
    cv::Point target_toolframePos1, target_toolframePos2;

    std::string T_topic = "/visual_servo/targets";

    std::vector<visual_servo::ToolDetector> detector_list{detector_toolcenter, detector_tooltip, detector_frametip};
    std::string G_topic = "/visual_servo/orientation_gradient";
    std::cout << "Done topic assignment" << std::endl;

    visual_servo::GradientUpdater G_updater(nh, G_topic, T_topic);
    // visual_servo::GradientUpdater G_updater(nh, G_topic, target_ori);

    std::cout << "Done updater initialization" << std::endl;

    G_updater.mainLoop(cam1, cam2, detector_list);

    ros::shutdown();
    return 0;

}