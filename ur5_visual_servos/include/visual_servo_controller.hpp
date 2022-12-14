#ifndef VISUAL_SERVO_CONTROLLER_H
#define VISUAL_SERVO_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>
#include <cfloat>
#include <sstream>
#include <vector>
#include <numeric>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "tool_detector.hpp"

namespace visual_servo{
    class VisualServoController{
        protected:

            ros::NodeHandle nh;

            int freq;
            int num_features;
            int dof;
            double tolerance;
            double K;
            double constJTh; // within which J remains a const (in pixs)
            // bool continueLoop;
            double servoMaxStep;

            // flags
            bool targetReceived;
            bool JChecked;
            bool continueLoop;

            Eigen::VectorXd toolPos;

            Eigen::VectorXd targets;
            Eigen::MatrixXd J;
            Eigen::VectorXd controlError;

            std::vector<double> J_flat;

            // ros subscribers
            ros::Subscriber target_sub;
            ros::Subscriber J_sub;

        public:
            // constructors
            VisualServoController(ros::NodeHandle& nh, double& tol, bool target_topic);
            // constructor with a fixed target
            VisualServoController(ros::NodeHandle& nh, double& tol, Eigen::VectorXd& targets);
            // destructor
            ~VisualServoController(){};
            // callbacks
            void JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            // optional
            void targetCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            // mutators
            void setServoMaxStep(int step);
            void setK(double K_);
            void setFreq(int f);
            // accessors
            bool stopSign();
            Eigen::VectorXd getToolPosition();
            // member functions 
            void directionIncrement(Eigen::VectorXd& inc, ImageCapturer& cam1, ImageCapturer& cam2, ToolDetector& detector);
            void directionIncrement(Eigen::VectorXd& inc, ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list);
            // utils
            void flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat);
            void limtVec(Eigen::VectorXd& v, double stepSize);

    };
}
#endif
