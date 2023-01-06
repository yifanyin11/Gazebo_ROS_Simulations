#ifndef TOOL_DETECTOR_H
#define TOOL_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_capturer.hpp"

namespace visual_servo{
    class ToolDetector{
    private:
        ros::NodeHandle nh;
        cv::Mat image;
        cv::Point tool_center;
        cv::Point corner1;
        cv::Point corner2;
        cv::Scalar lower_hsv;
        cv::Scalar upper_hsv;

    public:
        friend class JacobianUpdater; 
        friend class VisualServoController; 
        // constructor
        ToolDetector(ros::NodeHandle& nh, std::vector<int> hsv_range);
        // destructor
        ~ToolDetector(){};
        // mutators
        // accessors
        cv::Mat getSourceImage(ImageCapturer& cam);
        cv::Point getCenter();
        // functions
        void detect(ImageCapturer& cam); // update source image with the cur frame from cam, perform detection using that image, update tool_center
        void drawDetectRes(); 
    };
} // namespace visual_servo

#endif  