#include "pixel_picker.hpp"

PixelPicker::PixelPicker(double shrink_height_=0.75, int dot_radius_=5){
    shrink_height = shrink_height_;
    dot_radius = dot_radius_;
    point.x = -1;
    point.y = -1;
}


cv::Point PixelPicker::pickOne(cv::Mat img){
    // First create the image with alpha channel
    cv::cvtColor(img, rgba, cv::COLOR_RGB2RGBA);
    std::vector<cv::Mat> channels(4);
    // Then assign the mask to the last channel of the image
    cv::split(rgba, channels);
    // cv::Mat alpha(img.rows, img.cols, CV_8UC1, cv::Scalar(128));
    channels[3] = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(128));
    // modify channel// then merge
    cv::merge(channels, rgba);

    cv::namedWindow("img");

    rgba.copyTo(res);
    resshow = ResizeWithAspectRatio(res, -1, int(res.rows*shrink_height));
    cv::imshow("img", resshow);
    cv::setMouseCallback("img", PixelPicker::mouser, this);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return point;
}

// mutators
void PixelPicker::set_shrink_height(double shrink_height_){
    shrink_height = shrink_height_;
}

void PixelPicker::set_dot_radius(int dot_radius_){
    dot_radius = dot_radius_;
}

void PixelPicker::mouser(int event, int x, int y, int flags, void* this_) {
  static_cast<PixelPicker*>(this_)->onMouse(event, x, y);
}

void PixelPicker::onMouse(int evt, int x, int y) {
    rgba.copyTo(res);
    x = int(x/shrink_height);
    y = int(y/shrink_height);
    if(evt == cv::EVENT_LBUTTONDOWN) {
        point.x = x;
        point.y = y;
        cv::line(res, cv::Point(x, 0), cv::Point(x, res.rows), (0,0,255,128), 2);
        cv::line(res, cv::Point(0, y), cv::Point(res.cols, y), (0,0,255,128), 2);
    }
    // cnd = redDot[:, :, 3] > 0;
    // res[cnd] = redDot[cnd];
    resshow = ResizeWithAspectRatio(res, -1, int(res.rows*shrink_height));

    cv::imshow("img", resshow);
}

cv::Mat PixelPicker::ResizeWithAspectRatio(cv::Mat img, int width=-1, int height=-1){
    cv::Mat out;
    int h = img.rows;
    int w = img.cols;
    double r;

    if (width==-1 && height==-1){
        return img;
    }
    if (width==-1){
        r = 1.0*height/h;
        // dim = cv::Size(int(w * r), height);
        cv::resize(img, out, cv::Size(int(w * r), height), cv::INTER_AREA);
        return out;
    }
    else{
        r = 1.0*width/w;
        // dim = cv::Size(width, int(h * r));
        cv::resize(img, out, cv::Size(width, int(h * r)), cv::INTER_AREA);
        return out;
    }
}