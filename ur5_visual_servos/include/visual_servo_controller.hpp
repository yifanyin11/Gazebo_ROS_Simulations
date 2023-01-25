#ifndef VISUAL_SERVO_CONTROLLER_H
#define VISUAL_SERVO_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include<math.h>  

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
            double tolerance_ori;
            double K;
            double K_ori;
            double constJTh; // within which J remains a const (in pixs)
            double constJTh_ori;

            // bool continueLoop;
            double servoMaxStep;
            double servoAngMaxStep;

            // flags
            bool targetReceived;
            bool JChecked;
            bool GChecked;
            bool closeUpdateJOri;
            bool continueLoop;

            Eigen::VectorXd toolPos;
            Eigen::VectorXd toolRot;
            Eigen::VectorXd energy;

            Eigen::VectorXd targets;

            Eigen::MatrixXd J;
            Eigen::MatrixXd J_ori;

            Eigen::MatrixXd G_ori;

            Eigen::VectorXd controlError;
            Eigen::VectorXd controlErrorOri;

            std::vector<double> J_flat;
            std::vector<double> J_ori_flat;
            std::vector<double> G_ori_flat;

            // ros subscribers
            ros::Subscriber target_sub;
            ros::Subscriber J_sub;
            ros::Subscriber G_sub;

        public:
            // constructors
            VisualServoController(ros::NodeHandle& nh, bool target_topic, double tol, double tol_ori);
            // constructor with a fixed target
            VisualServoController(ros::NodeHandle& nh, Eigen::VectorXd& targets, double tol, double tol_ori);
            // destructor
            ~VisualServoController(){};
            // callbacks
            void JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void gradientCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
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
            void oriDirectionIncrement(Eigen::VectorXd& inc, ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list);
            void poseDirectionIncrement(Eigen::VectorXd& inc, ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list);
            // utils
            static void flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat);
            static void flat2eigenVec(Eigen::VectorXd& V, std::vector<double> flat);
            static void limInc(Eigen::VectorXd& v, double stepSize);
            static void getToolRot(Eigen::VectorXd& toolRot, cv::Point& center1, cv::Point& tooltip1, cv::Point& frametip1, cv::Point& center2, cv::Point& tooltip2, cv::Point& frametip2);
            static void stdAngControlError(Eigen::VectorXd& controlErrorOri);
            static void calculateEnergyFunction(Eigen::VectorXd delToolRot, Eigen::VectorXd& energy);
            static double getRotDis(Eigen::VectorXd& toolRot1, Eigen::VectorXd& toolRot2);
    };
}
#endif
