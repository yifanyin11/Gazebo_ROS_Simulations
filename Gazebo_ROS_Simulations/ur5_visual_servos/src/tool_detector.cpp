#include "tool_detector.hpp"

visual_servo::ToolDetector::ToolDetector(ros::NodeHandle& nh, std::vector<int> hsv_range):
nh(nh){
    tool_center.x = -1.0;
    tool_center.y = -1.0;
    corner1.x = -1.0;
    corner1.y = -1.0;
    corner2.x = -1.0;
    corner2.y = -1.0;
    lower_hsv = cv::Scalar(hsv_range[0], hsv_range[1], hsv_range[2]);
    upper_hsv = cv::Scalar(hsv_range[3], hsv_range[4], hsv_range[5]);
}

cv::Mat visual_servo::ToolDetector::getSourceImage(visual_servo::ImageCapturer& cam){
    if (cam.img_ptr->image.empty()){
        ROS_ERROR("No detection recorded yet.");
    }
    return cam.getCurrentImage();
}

cv::Point visual_servo::ToolDetector::getCenter(){
    return tool_center;
}

void visual_servo::ToolDetector::detect(visual_servo::ImageCapturer& cam){
    // update source image
    image = cam.getCurrentImage();
    // perform detection
    cv::Mat hsv, mask, col_sum, row_sum;
    // convert to hsv colorspace
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // find the red color within the boundaries
    cv::inRange(hsv, lower_hsv, upper_hsv, mask);

    // cv::namedWindow("mask");
    // cv::imshow("mask", mask);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, CV_64FC1); 
    cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, CV_64FC1); 
    int start1, end1, start2, end2;
    for (int i=0; i<col_sum.size[1]; ++i){
        if (col_sum.at<double>(0,i)!=0){
            start1 = i;
            break;
        }
    }
    for (int i=col_sum.size[1]-1; i>=0; --i){
        if (col_sum.at<double>(0,i)!=0){
            end1 = i;
            break;
        }
    }
    
    for (int i=0; i<row_sum.size[0]; ++i){
        if (row_sum.at<double>(i,0)!=0){
            start2 = i;
            break;
        }
    }
    for (int i=row_sum.size[0]-1; i>=0; --i){
        if (row_sum.at<double>(i,0)!=0){
            end2 = i;
            break;
        }
    }
    tool_center.x = (start1+end1)/2.0;
    tool_center.y = (start2+end2)/2.0;
    corner1.x = start1;
    corner1.y = end2;
    corner2.x = end1;
    corner2.y = start2;

    while(nh.ok()){
        ros::spinOnce();
        break;
    }
}

void visual_servo::ToolDetector::detect(cv::Mat& img){
    cv::Mat hsv, mask, col_sum, row_sum;
    // convert to hsv colorspace
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    // find the red color within the boundaries
    cv::inRange(hsv, lower_hsv, upper_hsv, mask);

    // cv::namedWindow("mask");
    // cv::imshow("mask", mask);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, CV_64FC1); 
    cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, CV_64FC1); 
    int start1, end1, start2, end2;
    for (int i=0; i<col_sum.size[1]; ++i){
        if (col_sum.at<double>(0,i)!=0){
            start1 = i;
            break;
        }
    }
    for (int i=col_sum.size[1]-1; i>=0; --i){
        if (col_sum.at<double>(0,i)!=0){
            end1 = i;
            break;
        }
    }
    
    for (int i=0; i<row_sum.size[0]; ++i){
        if (row_sum.at<double>(i,0)!=0){
            start2 = i;
            break;
        }
    }
    for (int i=row_sum.size[0]-1; i>=0; --i){
        if (row_sum.at<double>(i,0)!=0){
            end2 = i;
            break;
        }
    }
    tool_center.x = (start1+end1)/2.0;
    tool_center.y = (start2+end2)/2.0;
    corner1.x = start1;
    corner1.y = end2;
    corner2.x = end1;
    corner2.y = start2;

    while(nh.ok()){
        ros::spinOnce();
        break;
    }
}

void visual_servo::ToolDetector::drawDetectRes(){
    cv::Mat img = image.clone();
    cv::rectangle(img, corner1, corner2, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::circle(img, tool_center, 3, cv::Scalar(255, 0, 0), -1);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void visual_servo::ToolDetector::drawDetectRes(cv::Mat img_){
    cv::Mat img = img_.clone();
    cv::rectangle(img, corner1, corner2, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::circle(img, tool_center, 3, cv::Scalar(255, 0, 0), -1);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}