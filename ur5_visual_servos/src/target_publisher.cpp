#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

#include "visual_servo_controller.cpp"
#include "image_capturer.cpp"
#include "tool_detector.cpp"
#include "Jacobian_updater.cpp"

void vecEigen2std(Eigen::VectorXd& ve, std::vector<double>& vs){
    if (ve.size()!=vs.size()) vs.resize(ve.size());
    for (int i=0; i<ve.size(); ++i){
        vs[i] = ve(i);
    }
}

int main(int argc, char** argv){
    // Ros setups
    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::string target_topic = "/visual_servo/targets";
    ros::Publisher target_pub = nh.advertise<std_msgs::Float64MultiArray>(target_topic, 1);

    ros::Rate rate(10);
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

    int num_features = 4;
    int dof = 6;

    cv::Point target1, target2;
    cv::Point target_tooltipPos1, target_tooltipPos2;
    cv::Point target_toolframePos1, target_toolframePos2;

    Eigen::VectorXd targets, target_pos, target_ori;

    target_pos.resize(num_features);
    target_ori.resize(num_features);
    targets.resize(num_features*2);

    // target for position test
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

    target_pos << target1.x, target1.y, target2.x, target2.y;

    // target for orientation test 
    detector_target.detect(cam1);
    target1 = detector_target.getCenter();
    detector_target.drawDetectRes();
    detector_target.detect(cam2);
    target2 = detector_target.getCenter();
    detector_target.drawDetectRes();

    detector_target_tooltip.detect(cam1);
    target_tooltipPos1 = detector_target_tooltip.getCenter();
    detector_target_tooltip.drawDetectRes();
    detector_target_tooltip.detect(cam2);
    target_tooltipPos2 = detector_target_tooltip.getCenter();
    detector_target_tooltip.drawDetectRes();

    detector_target_frametip.detect(cam1);
    target_toolframePos1 = detector_target_frametip.getCenter();
    detector_target_frametip.drawDetectRes();
    detector_target_frametip.detect(cam2);
    target_toolframePos2 = detector_target_frametip.getCenter();
    detector_target_frametip.drawDetectRes();

    visual_servo::VisualServoController::getToolRot(target_ori, target1, target_tooltipPos1, target_toolframePos1, target2, target_tooltipPos2, target_toolframePos2);

    targets << target_pos, target_ori;

    std::cout << "Done initialize targets" << targets << std::endl;

    std_msgs::Float64MultiArray Tmsgs;

    std::vector<double> targets_std(targets.size(), 0.0);
    vecEigen2std(targets, targets_std);

    Tmsgs.data = targets_std;

    while(nh.ok()){
        target_pub.publish(Tmsgs);
        ros::spinOnce();
        rate.sleep();
    }

}
