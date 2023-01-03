#ifndef VISUAL_SERVO_CONTROLLER_H
#define VISUAL_SERVO_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <sanaria_msgs/VisualServoJointIncrement.h>
#include <sanaria_msgs/VisualServoPositions.h>
#include <sanaria_msgs/VisualServoGoal.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sanaria_calibration_tools/image_Jacobian_calculation.h>

#include <iostream>
#include <sstream>
#include <new>
#include <thread>
#include <vector>
#include <chrono>
#include <numeric>
#include <map>
#include <Eigen/Dense>

namespace visual_servo{
    class VisualServoController{
        protected:

            ros::NodeHandle nh;
            int freq;
            int num_features;
            int dof;
            double tolerance;
            double K;
            bool continueLoop;
            int update_pix_step;
            int update_enc_step;
            int servoMaxStep;
            int initStep;

            // flags
            bool targetReceived;
            bool JChecked;

            Eigen::VectorXd tipPosition;
            Eigen::VectorXd robotPosition;
            Eigen::VectorXd lastTipPos;
            Eigen::VectorXd lastRobotPos;
            Eigen::VectorXd targets;
            Eigen::MatrixXd J;
            Eigen::VectorXd controlError;

            std::vector<double> J_flat;

            // ros subscribers
            // ros::Subscriber encoder_subscriber;
            ros::Subscriber target_subscriber;
            ros::Subscriber tipPosition_subscriber;
            ros::Subscriber Jacobian_subscriber;

            // ros publishers
            ros::Publisher stopSignal_publisher;
            ros::Publisher direction_publisher;

        public:
            // constructors
            VisualServoController(ros::NodeHandle& nh, double& tol);
            // constructor with a fixed target
            VisualServoController(ros::NodeHandle& nh, double& tol, Eigen::VectorXd& targets);
            // destructor
            ~VisualServoController(){};
            // callbacks
            void JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            // void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg);
            void targetCallback(const sanaria_msgs::VisualServoGoal::ConstPtr& msg);
            void controlInputsCallback(const sanaria_msgs::VisualServoPositions::ConstPtr& msg);
            // mutators
            void setServoMaxStep(int step);
            void setUpdatePixStep(int step);
            void setUpdateEncStep(int step);
            void setK(double K_);
            void setFreq(int f);
            // accessors
            bool stopSign();
            Eigen::VectorXd getTipPosition();
            // member functions
            void mainLoop();
            void directionIncrement(Eigen::VectorXd& inc);
            void directionIncrementOverhead(Eigen::VectorXd& inc);
            // utils
            void flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat);
            void limtVec(Eigen::VectorXd& v, int stepSize);

    };
}
#endif
