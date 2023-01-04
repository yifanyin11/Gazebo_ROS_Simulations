#ifndef IMAGE_CAPTURER_H
#define IMAGE_CAPTURER_H

#include <ros/ros.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstdlib>
#include <iostream>
#include <ctime>

namespace visual_servo{
    class ImageCapturer{
    private:
        cv_bridge::CvImagePtr img_ptr;
        ros::NodeHandle nh;
        ros::Subscriber img_sub;
        std::string image_topic;
    public:
        friend class ToolDetector; 

        int count;
        // constructors
        ImageCapturer(const ImageCapturer& obj) = default;
        ImageCapturer(ros::NodeHandle& nh, std::string& img_topic);
        // destructor
        ~ImageCapturer(){};
        // ImageCapturer operator = (ImageCapturer &obj);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        cv::Mat getCurrentImage();
        void saveCurrentImage(std::string imgPath, std::string imgName);
    };
}

#endif