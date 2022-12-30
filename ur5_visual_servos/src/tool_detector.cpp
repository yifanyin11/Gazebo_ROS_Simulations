#include "tool_detector.hpp"

visual_servo::ToolDetector::ToolDetector(ImageCapturer& img_capturer):
cam(img_capturer){

}

cv::Mat visual_servo::ToolDetector::getSourceImage(){
    return image;
}

cv::Point visual_servo::ToolDetector::getToolCenter(){
    return tool_center;
}

void visual_servo::ToolDetector::detect(){
    // update source image
    image = cam.getCurrentImage();
    cv::Mat img = getSourceImage();
    // perform detection
    cv::Mat hsv, mask, col_sum, row_sum;
    // convert to hsv colorspace
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    // find the red color within the boundaries
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask);
    // // define kernel size  
    // kernel = np.ones((15,15),np.uint8)
    // // Remove unnecessary noise from mask
    // mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    // mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, -1); 
    cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, -1); 
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
    corner1.y = start2;
    corner2.x = end1;
    corner2.y = end2;
}

void visual_servo::ToolDetector::drawDetectRes(){
    cv::Mat img = getSourceImage();
    cv::rectangle(img, corner1, corner2, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::circle(img, tool_center, 3, cv::Scalar(0, 0, 255), -1);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}