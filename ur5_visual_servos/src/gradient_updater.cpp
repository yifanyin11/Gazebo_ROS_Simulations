#include "gradient_updater.hpp"

visual_servo::GradientUpdater::GradientUpdater(ros::NodeHandle& nh, std::string& G_topic_, Eigen::VectorXd& target_ori_):
nh(nh){
    
    // initializations
    target_ori = target_ori_;

    G_topic = G_topic_;
    dof_robot = 3;
    num_features = 4;

    update_enc_ang_step = M_PI/12;

    initStepOri = 0.15;

    energy.resize(num_features);
    lastEnergy.resize(num_features);
    toolRot.resize(num_features);
    lastToolRot.resize(num_features);
    robotRot.resize(dof_robot);
    lastRobotRot.resize(dof_robot);

    G_ori.resize(num_features, dof_robot);
    G_ori_flat.resize(dof_robot*num_features);

    // publishers
    G_pub = nh.advertise<std_msgs::Float64MultiArray>(G_topic, 1);
}

void visual_servo::GradientUpdater::evalCostFunction(const double *params, int num_inputs, const void *inputs, double *fvec, int *info)
{
    const OptimDataG* optim_data = reinterpret_cast<const OptimDataG*>(inputs);
    Eigen::MatrixXd G_cur;
    int dof_robot = optim_data->del_r.size();
    int num_features = optim_data->del_Pr.size();

    G_cur.resize(num_features, dof_robot);
    flat2eigen(G_cur, params);

    Eigen::VectorXd error1 = (G_cur*optim_data->del_r) - (optim_data->del_Pr);
    Eigen::VectorXd error2 = (G_cur.transpose()*G_cur).inverse()*G_cur.transpose()*(optim_data->del_Pr) - (optim_data->del_r);

    for (int i = 0; i<num_features; ++i) {
        fvec[i] = abs(error1[i]);
    }
    for (int i = num_features; i<dof_robot+num_features; ++i) {
        fvec[i] = abs(error2[i-num_features]);
    }
    // fvec[dof_robot+num_features] = abs(J_cur.norm()-optim_data->J_norm);
    for (int i = dof_robot+num_features; i<num_inputs; ++i) {
        fvec[i] = 0.0;
    }

}

bool visual_servo::GradientUpdater::runLM(const OptimDataG& optim_data, const std::vector<double>& initial_state, std::vector<double>& result)
{
    const size_t dof = initial_state.size();
    const size_t m = optim_data.getM();

    double* params = new double[dof];

    for (size_t i = 0; i < dof; i ++) params[i] = initial_state[i];

    // LM solver buffers
    double* LM_fvec = new double[m];
    double* LM_diag = new double[dof];
    double* LM_qtf  = new double[dof];
    double* LM_fjac = new double[dof * m];
    double* LM_wa1  = new double[dof];
    double* LM_wa2  = new double[dof];
    double* LM_wa3  = new double[dof];
    double* LM_wa4  = new double[m];
    int*    LM_ipvt = new int [dof];

    double epsilon = 1e-20;
    int maxcall = 100;

    lm_control_struct control;
    control = lm_control_double;
    control.maxcall = maxcall;
    control.ftol = 1e-20;
    control.xtol = 1e-20;
    control.gtol = 1e-20;
    control.epsilon = epsilon;
//    control.stepbound = 100.0;
    control.printflags = 0;

    lm_status_struct status;
    status.info = 1;

    if (!control.scale_diag) {
        for (size_t j = 0; j < dof; j ++) LM_diag[j] = 1;
    }

    lm_lmdif(static_cast<int>(m),
             static_cast<int>(dof),
             params,
             LM_fvec,
             control.ftol,
             control.xtol,
             control.gtol,
             control.maxcall * static_cast<int>(dof + 1),
             control.epsilon,
             LM_diag,
             (control.scale_diag ? 1 : 2),
             control.stepbound,
             &(status.info),
             &(status.nfev),
             LM_fjac,
             LM_ipvt,
             LM_qtf,
             LM_wa1,
             LM_wa2,
             LM_wa3,
             LM_wa4,
             evalCostFunction,
             lm_printout_std,
             control.printflags,
             &optim_data);

    result.resize(dof);
    for (size_t i = 0; i < dof; i ++) result[i] = params[i];

    delete [] LM_fvec;
    delete [] LM_diag;
    delete [] LM_qtf;
    delete [] LM_fjac;
    delete [] LM_wa1;
    delete [] LM_wa2;
    delete [] LM_wa3;
    delete [] LM_wa4;
    delete [] LM_ipvt;
    delete [] params;

    return true;
}

void visual_servo::GradientUpdater::initialize(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, std::vector<visual_servo::ToolDetector>& detector_list){
    // Ros setups
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // MOVEIT planning setups
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setMaxVelocityScalingFactor(0.3);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.01);
    bool success;

    // // Move to home position
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // define points
    cv::Point tool_center1, tool_center2; 

    cv::Point tooltip_drl1, tooltip_drr1, tooltip_dpl1, tooltip_dpr1, tooltip_dyl1, tooltip_dyr1; // tooltip for cam1
    cv::Point toolframe_drl1, toolframe_drr1, toolframe_dpl1, toolframe_dpr1, toolframe_dyl1, toolframe_dyr1; // tool frame tip for cam1

    cv::Point tooltip_drl2, tooltip_drr2, tooltip_dpl2, tooltip_dpr2, tooltip_dyl2, tooltip_dyr2; // tooltip for cam2
    cv::Point toolframe_drl2, toolframe_drr2, toolframe_dpl2, toolframe_dpr2, toolframe_dyl2, toolframe_dyr2; // tool frame tip for cam2
    
    // get current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    // define target pose and initialize it as current pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position = current_pose.pose.position;

    // define variables for transform calculation
    double roll, pitch, yaw;
    tf::Quaternion q_target;
    tf::Transform transform_target;
    visual_servo::GradientUpdater::poseMsg2Transform(transform_target, target_pose);
    tf::Matrix3x3(transform_target.getRotation()).getRPY(roll, pitch, yaw);

    // get the fixed tool center points under both camera views
    detector_list[0].detect(cam1);
    tool_center1 = detector_list[0].getCenter();
    detector_list[0].detect(cam2);
    tool_center2 = detector_list[0].getCenter();

    // move to r-
    q_target.setRPY(roll-initStepOri, pitch, yaw);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);
    
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to r- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at r-
    detector_list[1].detect(cam1);
    tooltip_drl1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_drl1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_drl2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_drl2 = detector_list[2].getCenter();

    // move to r+
    q_target.setRPY(roll+initStepOri, pitch, yaw);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to r+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at r+
    detector_list[1].detect(cam1);
    tooltip_drr1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_drr1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_drr2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_drr2 = detector_list[2].getCenter();
    
    // move to p-
    q_target.setRPY(roll, pitch-initStepOri, yaw);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to p- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at p-
    detector_list[1].detect(cam1);
    tooltip_dpl1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_dpl1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_dpl2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_dpl2 = detector_list[2].getCenter();

    // move to p+
    q_target.setRPY(roll, pitch+initStepOri, yaw);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to p+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at p+
    detector_list[1].detect(cam1);
    tooltip_dpr1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_dpr1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_dpr2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_dpr2 = detector_list[2].getCenter();

    // move to y-
    q_target.setRPY(roll, pitch, yaw-initStepOri);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to y- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at y-
    detector_list[1].detect(cam1);
    tooltip_dyl1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_dyl1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_dyl2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_dyl2 = detector_list[2].getCenter();

    // move to y+
    q_target.setRPY(roll, pitch, yaw+initStepOri);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to y+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at y+
    detector_list[1].detect(cam1);
    tooltip_dyr1 = detector_list[1].getCenter();
    detector_list[2].detect(cam1);
    toolframe_dyr1 = detector_list[2].getCenter();

    detector_list[1].detect(cam2);
    tooltip_dyr2 = detector_list[1].getCenter();
    detector_list[2].detect(cam2);
    toolframe_dyr2 = detector_list[2].getCenter();

    // go back to center
    q_target.setRPY(roll, pitch, yaw);
    q_target.normalize();
    transform_target.setRotation(q_target);

    visual_servo::GradientUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_G", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    // calculate G
    // define delta thetas
    Eigen::VectorXd tool_ori_drl, tool_ori_dpl, tool_ori_dyl, tool_ori_drr, tool_ori_dpr, tool_ori_dyr;
    tool_ori_drl.resize(num_features);
    tool_ori_dpl.resize(num_features);
    tool_ori_dyl.resize(num_features);
    tool_ori_drr.resize(num_features);
    tool_ori_dpr.resize(num_features);
    tool_ori_dyr.resize(num_features);

    Eigen::VectorXd energy_rl, energy_rr, energy_pl, energy_pr, energy_yl, energy_yr;
    energy_rl.resize(num_features);
    energy_rr.resize(num_features);
    energy_pl.resize(num_features);
    energy_pr.resize(num_features);
    energy_yl.resize(num_features);
    energy_yr.resize(num_features);
    std::cout << "start gettig tool rot" << std::endl;
    
    visual_servo::GradientUpdater::getToolRot(tool_ori_drl, tool_center1, tooltip_drl1, toolframe_drl1, tool_center2, tooltip_drl2, toolframe_drl2);
    visual_servo::GradientUpdater::getToolRot(tool_ori_drr, tool_center1, tooltip_drr1, toolframe_drr1, tool_center2, tooltip_drr2, toolframe_drr2);
    visual_servo::GradientUpdater::getToolRot(tool_ori_dpl, tool_center1, tooltip_dpl1, toolframe_dpl1, tool_center2, tooltip_dpr2, toolframe_dpl2);
    visual_servo::GradientUpdater::getToolRot(tool_ori_dpr, tool_center1, tooltip_dpr1, toolframe_dpr1, tool_center2, tooltip_dpr2, toolframe_dpr2);
    visual_servo::GradientUpdater::getToolRot(tool_ori_dyl, tool_center1, tooltip_dyl1, toolframe_dyl1, tool_center2, tooltip_dyl2, toolframe_dyl2);
    visual_servo::GradientUpdater::getToolRot(tool_ori_dyr, tool_center1, tooltip_dyr1, toolframe_dyr1, tool_center2, tooltip_dyr2, toolframe_dyr2);

    std::cout << "start getting energy" << std::endl;

    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_drl-target_ori, energy_rl);
    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_drr-target_ori, energy_rr);
    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_dpl-target_ori, energy_pl);
    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_dpr-target_ori, energy_pr);
    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_dyl-target_ori, energy_yl);
    visual_servo::GradientUpdater::calculateEnergyFunction(tool_ori_dyr-target_ori, energy_yr);

    std::cout << "done getting energy" << std::endl;

    G_ori_flat[0]=(energy_rr(0)-energy_rl(0))/(2*initStepOri);
    G_ori_flat[1]=(energy_pr(0)-energy_pl(0))/(2*initStepOri);
    G_ori_flat[2]=(energy_yr(0)-energy_yl(0))/(2*initStepOri);
    G_ori_flat[3]=(energy_rr(1)-energy_rl(1))/(2*initStepOri);
    G_ori_flat[4]=(energy_pr(1)-energy_pl(1))/(2*initStepOri);
    G_ori_flat[5]=(energy_yr(1)-energy_yl(1))/(2*initStepOri);
    G_ori_flat[6]=(energy_rr(2)-energy_rl(2))/(2*initStepOri);
    G_ori_flat[7]=(energy_pr(2)-energy_pl(2))/(2*initStepOri);
    G_ori_flat[8]=(energy_yr(2)-energy_yl(2))/(2*initStepOri);
    G_ori_flat[9]=(energy_rr(3)-energy_rl(3))/(2*initStepOri);
    G_ori_flat[10]=(energy_pr(3)-energy_pl(3))/(2*initStepOri);
    G_ori_flat[11]=(energy_yr(3)-energy_yl(3))/(2*initStepOri);

    std::cout << "done calculating gradient" << std::endl;

    visual_servo::GradientUpdater::flat2eigen(G_ori, G_ori_flat);
    std::cout << "G: " << G_ori << std::endl;
}

void visual_servo::GradientUpdater::updateGradient(Eigen::VectorXd& del_Pr, Eigen::VectorXd& del_r){
    OptimDataG data;
    data.del_Pr = del_Pr;
    data.del_r = del_r;
    data.G_norm = G_ori.norm();
    std::vector<double> result(G_ori_flat.size());
    runLM(data, G_ori_flat, result);
    G_ori_flat = result;
    flat2eigen(G_ori, result);
}

void visual_servo::GradientUpdater::mainLoop(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, std::vector<visual_servo::ToolDetector> & detector_list){
    ros::Rate rate(10.0);

    initialize(cam1, cam2, detector_list);

    std_msgs::Float64MultiArray Gmsg;
    cv::Point toolPos1, toolPos2;
    cv::Point tooltipPos1, tooltipPos2;
    cv::Point toolframePos1, toolframePos2;

    double roll, pitch, yaw;

    cv::Mat image1, image2;

    ROS_INFO("Gradient initialized! Ready for gradient update.");
    // loopup current robot pose
    while (nh.ok()){
        try{
            image1 = cam1.getCurrentImage();
            image2 = cam2.getCurrentImage();

            listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);
            break;
            }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            // rate.sleep();
            }   
    }

    detector_list[0].detect(image1);
    toolPos1 = detector_list[0].getCenter();
    detector_list[0].detect(image2);
    toolPos2 = detector_list[0].getCenter();

    detector_list[1].detect(image1);
    tooltipPos1 = detector_list[1].getCenter();
    detector_list[1].detect(image2);
    tooltipPos2 = detector_list[1].getCenter();

    detector_list[2].detect(image1);
    toolframePos1 = detector_list[2].getCenter();
    detector_list[2].detect(image2);
    toolframePos2 = detector_list[2].getCenter();

    visual_servo::GradientUpdater::getToolRot(lastToolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);
    visual_servo::GradientUpdater::calculateEnergyFunction(lastToolRot-target_ori, lastEnergy);
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    lastRobotRot << roll, pitch, yaw;

    Eigen::VectorXd encAngDisplFromLast;
    Eigen::VectorXd energyDiffFromLast;

    while ((nh.ok())){
        while (nh.ok()){
            try{
                //*** TODO ***
                // overload detect function to detect a given image, and only capture images in this loop
                image1 = cam1.getCurrentImage();
                image2 = cam2.getCurrentImage();

                listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);

                break;
                }
            catch (tf::TransformException ex){
                // ROS_ERROR("%s",ex.what());
                }   
        }
        
        detector_list[0].detect(image1);
        toolPos1 = detector_list[0].getCenter();
        detector_list[0].detect(image2);
        toolPos2 = detector_list[0].getCenter();

        detector_list[1].detect(image1);
        tooltipPos1 = detector_list[1].getCenter();
        detector_list[1].detect(image2);
        tooltipPos2 = detector_list[1].getCenter();

        detector_list[2].detect(image1);
        toolframePos1 = detector_list[2].getCenter();
        detector_list[2].detect(image2);
        toolframePos2 = detector_list[2].getCenter();

        visual_servo::GradientUpdater::getToolRot(toolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);
        visual_servo::GradientUpdater::calculateEnergyFunction(toolRot-target_ori, energy);
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
        robotRot << roll, pitch, yaw;

        std::cout << "toolRotation: " << toolRot <<std::endl;
        std::cout << "featureAngDisFromLast: " << toolRot-lastToolRot <<std::endl;

        encAngDisplFromLast = robotRot-lastRobotRot;
        std::cout << "robotRotation: " << robotRot <<std::endl;
        std::cout << "encDisFromLast: " << encAngDisplFromLast << std::endl;

        energyDiffFromLast = energy-lastEnergy;
        std::cout << "energy: " << energy << std::endl;
        std::cout << "energyDiffFromLast: " << energyDiffFromLast << std::endl;

        std::cout << "G_ori: " << G_ori << std::endl;
        std::cout << "\n" << std::endl;
        

        // update G_ori only if greater than a distance from the last update
        if (encAngDisplFromLast.norm()>= update_enc_ang_step){
            updateGradient(energyDiffFromLast, encAngDisplFromLast);
            lastEnergy = energy;
            lastToolRot = toolRot;
            lastRobotRot = robotRot;
        } 

        Gmsg.data = G_ori_flat;
        G_pub.publish(Gmsg);
        ros::spinOnce();
        rate.sleep();
    }
}


// #################
// ##    Utils    ##
// #################

void visual_servo::GradientUpdater::flat2eigen(Eigen::MatrixXd& M, const double* flat){
    int size_M = M.cols()*M.rows();

    int cur = 0;
    for (int i=0; i<M.rows(); ++i){
        for (int j=0; j<M.cols(); ++j){
            M.coeffRef(i,j) = flat[cur];
            cur++;
        }
    }
}

void visual_servo::GradientUpdater::flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat){
    // check input
    int size_flat = flat.size();
    int size_M = M.cols()*M.rows();
    if (size_flat!=size_M){
        ROS_ERROR("Dimension inconsistent! Cannot convert to Eigen matrix.");
        return;
    }
    int cur = 0;
    for (int i=0; i<M.rows(); ++i){
        for (int j=0; j<M.cols(); ++j){
            M.coeffRef(i,j) = flat[cur];
            cur++;
        }
    }
}

void visual_servo::GradientUpdater::transform2PoseMsg(tf::Transform& transform, geometry_msgs::Pose& pose){
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();

    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
}

void visual_servo::GradientUpdater::poseMsg2Transform(tf::Transform& transform, geometry_msgs::Pose& pose){
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
}

void visual_servo::GradientUpdater::getToolRot(Eigen::VectorXd& toolRot, cv::Point& center1, cv::Point& tooltip1, cv::Point& frametip1, cv::Point& center2, cv::Point& tooltip2, cv::Point& frametip2){
    double theta11 = atan2((tooltip1-center1).y, (tooltip1-center1).x); 
    double theta12 = atan2((frametip1-center1).y, (frametip1-center1).x); 
    double theta21 = atan2((tooltip2-center2).y, (tooltip2-center2).x); 
    double theta22 = atan2((frametip2-center2).y, (frametip2-center2).x); 
    toolRot << theta11, theta12, theta21, theta22;
}

void visual_servo::GradientUpdater::calculateEnergyFunction(Eigen::VectorXd delToolRot, Eigen::VectorXd& energy){
    for (int i=0; i<energy.size(); ++i){
        energy(i)=sin(delToolRot(i)/2.0);
    }
}