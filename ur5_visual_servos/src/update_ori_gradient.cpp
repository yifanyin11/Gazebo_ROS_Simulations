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

    Eigen::VectorXd target_ori;

    target_ori.resize(num_features);
    target_ori << 1.5708, 2.63449, 1.5708, -2.98894;

    // detector_target.detect(cam1);
    // target1 = detector_target.getCenter();
    // detector_target.detect(cam2);
    // target2 = detector_target.getCenter();

    // detector_target_tooltip.detect(cam1);
    // target_tooltipPos1 = detector_target_tooltip.getCenter();
    // detector_target_tooltip.detect(cam2);
    // target_tooltipPos2 = detector_target_tooltip.getCenter();

    // detector_target_frametip.detect(cam1);
    // target_toolframePos1 = detector_target_frametip.getCenter();
    // detector_target_frametip.detect(cam2);
    // target_toolframePos2 = detector_target_frametip.getCenter();

    // detector_target.detect(cam1);
    // target1 = detector_target.getCenter();
    // detector_target.drawDetectRes();
    // detector_target.detect(cam2);
    // target2 = detector_target.getCenter();
    // detector_target.drawDetectRes();

    // detector_target_tooltip.detect(cam1);
    // target_tooltipPos1 = detector_target_tooltip.getCenter();
    // detector_target_tooltip.drawDetectRes();
    // detector_target_tooltip.detect(cam2);
    // target_tooltipPos2 = detector_target_tooltip.getCenter();
    // detector_target_tooltip.drawDetectRes();

    // detector_target_frametip.detect(cam1);
    // target_toolframePos1 = detector_target_frametip.getCenter();
    // detector_target_frametip.drawDetectRes();
    // detector_target_frametip.detect(cam2);
    // target_toolframePos2 = detector_target_frametip.getCenter();
    // detector_target_frametip.drawDetectRes();

    // std::cout << "Done detections" << std::endl;

    // visual_servo::GradientUpdater::getToolRot(target_ori, target1, target_tooltipPos1, target_toolframePos1, target2, target_tooltipPos2, target_toolframePos2);
    std::cout << "Done initialize target_ori" << target_ori << std::endl;
    std::cout << "Done target assignment" << std::endl;

    std::vector<visual_servo::ToolDetector> detector_list{detector_toolcenter, detector_tooltip, detector_frametip};
    std::string G_topic = "/visual_servo/orientation_gradient";
    std::cout << "Done topic assignment" << std::endl;


    visual_servo::GradientUpdater G_updater(nh, G_topic, target_ori);

    std::cout << "Done updater initialization" << std::endl;

    G_updater.mainLoop(cam1, cam2, detector_list);

    ros::shutdown();
    return 0;

}