#include "image_capturer.hpp"

visual_servo::ImageCapturer::ImageCapturer(ros::NodeHandle& nh, std::string& img_topic) : 
nh(nh), image_topic(img_topic){
    img_sub = nh.subscribe(image_topic, 1, &visual_servo::ImageCapturer::imageCallback, this);
    while(nh.ok()){
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        break;
    }
}

void visual_servo::ImageCapturer::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
            // for test only
            count++;
            std::cout << "count in callback: " << count << std::endl;

            img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::namedWindow("raw");
            cv::imshow("raw", img_ptr->image);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.\n", msg->encoding.c_str());
        }
}

cv::Mat visual_servo::ImageCapturer::getCurrentImage(){
    ros::Rate rate(100);
    while(nh.ok()){
        ros::spinOnce();
        std::cout << "count outside callback: " << count << std::endl;
        cv::namedWindow("get");
        cv::imshow("get", img_ptr->image);
        cv::waitKey(0);
        cv::destroyAllWindows();
        return (img_ptr->image).clone();
    }
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
