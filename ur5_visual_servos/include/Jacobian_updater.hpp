#ifndef JACOBIAN_UPDATER_H
#define JACOBIAN_UPDATER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "lm/lmmin.h"
#include "lm/lmmin.cpp"
#include "tool_detector.hpp"

namespace visual_servo{
    struct OptimData
    {
        size_t getM() const
        {
            return del_r.size()*del_Pr.size();
        }
        Eigen::VectorXd del_Pr;
        Eigen::VectorXd del_r;
    }; 

    class JacobianUpdater{
    private:
        ros::NodeHandle nh;
        std::string J_topic;

        int dof_robot;
        int num_features;

        int update_pix_step;
        int update_enc_step;
        int servoMaxStep;
        int initStep;

        bool toolPosReceived;
        bool encReceived;

        Eigen::VectorXd toolPosition;
        Eigen::VectorXd lastToolPos;
        Eigen::VectorXd robotPosition;
        Eigen::VectorXd lastRobotPos;
        Eigen::MatrixXd J;
        std::vector<double> J_flat;

        // publishers
        ros::Publisher J_pub;

    public:
        // constructors
        JacobianUpdater(ros::NodeHandle& nh, std::string& toolPos_topic);
        // destructor
        ~JacobianUpdater(){};
        // static functions for optimization
        static void evalCostFunction(const double *params, int num_inputs, const void *inputs, double *fvec, int *info);
        static bool runLM(const OptimData& optim_data, const std::vector<double>& initial_state, std::vector<double>& result);
        // initialization function
        int initializeJacobian(ImageCapturer& cam, ToolDetector& detector, visual_servo::ToolDetector& detector, int initStep=0.05);
        // update funcion
        void updateJacobian(Eigen::VectorXd& del_Px, Eigen::VectorXd& del_x);
        // main loop
        void mainLoop(ToolDetector& detector);
        // utils
        static void flat2eigen(Eigen::MatrixXd& M, const double* flat);
        static void flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat);
    };

}

#endif