#ifndef PIXEL_PICKER_H
#define PIXEL_PICKER_H

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class PixelPicker{
private:
    double shrink_height;
    int dot_radius;
    cv::Mat res;
    cv::Mat rgba;
    cv::Mat resshow;
    cv::Mat redDot;
    cv::Point point;

public:
    PixelPicker(double shrink_height_, int dot_radius_);
    cv::Point pickOne(cv::Mat img);
    // mutators
    void set_shrink_height(double shrink_height_);
    void set_dot_radius(int dot_radius_);
    void onMouse(int evt, int x, int y);
    static void mouser(int evt, int x, int y, int flags, void* this_);
    cv::Mat ResizeWithAspectRatio(cv::Mat img, int width, int height);
};


#endif