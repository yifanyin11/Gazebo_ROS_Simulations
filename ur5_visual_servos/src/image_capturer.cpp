#include "image_capturer.hpp"

visual_servo::ImageCapturer::ImageCapturer(ros::NodeHandle& nh, std::string& img_topic) : 
nh(nh), image_topic(img_topic){
    img_sub = nh.subscribe(image_topic, 1, &ImageCapturer::imageCallback, this);
}

void visual_servo::ImageCapturer::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
            img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.\n", msg->encoding.c_str());
        }
}

cv::Mat visual_servo::ImageCapturer::getCurrentImage(){
    return img_ptr->image;
}

void visual_servo::ImageCapturer::saveCurrentImage(std::string imgPath, std::string imgName){
    std::string img_full_path = imgPath+imgName;
    if(!(img_ptr->image.empty())){
        if (!cv::imwrite(img_full_path, img_ptr->image)){
            std::cout << "Image cannot be saved as '" << img_full_path << "'." << std::endl;
        }
        else{
            std::cout << "Image saved in '" << img_full_path << "'." << std::endl;
        }
    }
    else{
        ROS_ERROR("Fail to capture image.\n");
    }
}
