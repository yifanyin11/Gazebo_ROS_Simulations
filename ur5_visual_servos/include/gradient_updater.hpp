#ifndef GRADIENT_UPDATER_H
#define GRADIENT_UPDATER_H

#include <ros/ros.h>
#include <math.h>  

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

#include "lmmin.h"
#include "lmmin.cpp"
#include "tool_detector.hpp"

namespace visual_servo{
    struct OptimDataG
    {
        size_t getM() const
        {
            return del_r.size()*del_Pr.size();
        }
        Eigen::VectorXd del_Pr;
        Eigen::VectorXd del_r;
        double G_norm;
    }; 

    class GradientUpdater{
    private:
        ros::NodeHandle nh;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        std::string G_topic;

        int dof_robot;
        int num_features;

        double update_enc_ang_step;

        double initStepOri;
        
        Eigen::VectorXd target_ori;

        Eigen::VectorXd energy;
        Eigen::VectorXd lastEnergy;
        Eigen::VectorXd toolRot;
        Eigen::VectorXd lastToolRot;
        Eigen::VectorXd robotRot;
        Eigen::VectorXd lastRobotRot;

        Eigen::MatrixXd G_ori;
        std::vector<double> G_ori_flat;

        // publishers
        ros::Publisher G_pub;

    public:
        // constructors
        GradientUpdater(ros::NodeHandle& nh, std::string& toolPos_topic, Eigen::VectorXd& target_ori);
        // destructor
        ~GradientUpdater(){};
        // static functions for optimization
        static void evalCostFunction(const double *params, int num_inputs, const void *inputs, double *fvec, int *info);
        static bool runLM(const OptimDataG& optim_data, const std::vector<double>& initial_state, std::vector<double>& result);
        // initialization function
        void initialize(ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list);
        // update funcion
        void updateGradient(Eigen::VectorXd& del_Pr, Eigen::VectorXd& del_r);
        // main loop
        void mainLoop(ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list);
        // utils
        static void flat2eigen(Eigen::MatrixXd& M, const double* flat);
        static void flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat);
        static void transform2PoseMsg(tf::Transform& transform, geometry_msgs::Pose& pose);
        static void poseMsg2Transform(tf::Transform& transform, geometry_msgs::Pose& pose);
        static void getToolRot(Eigen::VectorXd& toolRot, cv::Point& center1, cv::Point& tooltip1, cv::Point& frametip1, cv::Point& center2, cv::Point& tooltip2, cv::Point& frametip2);
        static void calculateEnergyFunction(Eigen::VectorXd delToolRot, Eigen::VectorXd& energy);
    };

}

#endif