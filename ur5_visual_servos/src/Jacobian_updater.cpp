#include "Jacobian_updater.hpp"

visual_servo::JacobianUpdater::JacobianUpdater(ros::NodeHandle& nh, std::string& J_topic_):
nh(nh){
    
    // initializations
    J_topic = J_topic_;
    dof_robot = 3;
    num_features = 4;

    update_pix_step = 50;
    update_enc_step = 0.3;

    update_pix_ang_step = M_PI/4;
    update_enc_ang_step = M_PI/4;

    initStep = 0.05;
    initStepOri = M_PI/12;

    toolPos.resize(num_features);
    lastToolPos.resize(num_features);
    robotPos.resize(dof_robot);
    lastRobotPos.resize(dof_robot);

    toolRot.resize(num_features);
    lastToolRot.resize(num_features);
    robotRot.resize(dof_robot);
    lastRobotRot.resize(dof_robot);

    J.resize(num_features, dof_robot);
    J_ori.resize(num_features, dof_robot);
    J_flat.resize(dof_robot*num_features);
    J_ori_flat.resize(dof_robot*num_features);

    // publishers
    J_pub = nh.advertise<std_msgs::Float64MultiArray>(J_topic, 1);
}

void visual_servo::JacobianUpdater::evalCostFunction(const double *params, int num_inputs, const void *inputs, double *fvec, int *info)
{
    const OptimData* optim_data = reinterpret_cast<const OptimData*>(inputs);
    Eigen::MatrixXd J_cur;
    int dof_robot = optim_data->del_r.size();
    int num_features = optim_data->del_Pr.size();
    // int num_cams = num_features/2;
    J_cur.resize(num_features, dof_robot);
    flat2eigen(J_cur, params);
    Eigen::VectorXd error1 = (J_cur*optim_data->del_r) - (optim_data->del_Pr);
    Eigen::VectorXd error2 = (J_cur.transpose()*J_cur).inverse()*J_cur.transpose()*(optim_data->del_Pr) - (optim_data->del_r);
    int repeat = optim_data->del_r.size();
    for (int i = 0; i<num_features; ++i) {
        fvec[i] = error1[i];
    }
    for (int i = num_features; i<dof_robot+num_features; ++i) {
        fvec[i] = error2[i-num_features];
    }
    for (int i = dof_robot+num_features; i<num_inputs; ++i) {
        fvec[i] = 0.0;
    }

}

bool visual_servo::JacobianUpdater::runLM(const OptimData& optim_data, const std::vector<double>& initial_state, std::vector<double>& result)
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

void visual_servo::JacobianUpdater::initializeJacobian(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, visual_servo::ToolDetector& detector){
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

    // Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // define points
    cv::Point tool_dxl1, tool_dyl1, tool_dzl1, tool_dxr1, tool_dyr1, tool_dzr1;
    cv::Point tool_dxl2, tool_dyl2, tool_dzl2, tool_dxr2, tool_dyr2, tool_dzr2;
    // get current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    // define target pose and initialize it as current pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position = current_pose.pose.position;
    // move to x-
    target_pose.position.x = current_pose.pose.position.x-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to x- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at x-
    detector.detect(cam1);
    tool_dxl1 = detector.getCenter();
    detector.detect(cam2);
    tool_dxl2 = detector.getCenter();
    // move to x+
    target_pose.position.x = target_pose.position.x+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to x+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at x+
    detector.detect(cam1);
    tool_dxr1 = detector.getCenter();
    detector.detect(cam2);
    tool_dxr2 = detector.getCenter();
    
    // move to y-
    target_pose.position.x = target_pose.position.x-initStep;
    target_pose.position.y = target_pose.position.y-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to y- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at y-
    detector.detect(cam1);
    tool_dyl1 = detector.getCenter();
    detector.detect(cam2);
    tool_dyl2 = detector.getCenter();

    // move to y+
    target_pose.position.y = target_pose.position.y+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to y+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at x+
    detector.detect(cam1);
    tool_dyr1 = detector.getCenter();
    detector.detect(cam2);
    tool_dyr2 = detector.getCenter();

    // move to z-
    target_pose.position.y = target_pose.position.y-initStep;
    target_pose.position.z = target_pose.position.z-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to z- finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at z-
    detector.detect(cam1);
    tool_dzl1 = detector.getCenter();
    detector.detect(cam2);
    tool_dzl2 = detector.getCenter();

    // move to z+
    target_pose.position.z = target_pose.position.z+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    std::cout << "move to z+ finished ..." << std::endl;
    ros::Duration(0.5).sleep();

    // detect tool image position at z+
    detector.detect(cam1);
    tool_dzr1 = detector.getCenter();
    detector.detect(cam2);
    tool_dzr2 = detector.getCenter();

    // go back to center
    target_pose.position.z = target_pose.position.z-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    // calculate J
    double du1overdx = (tool_dxr1.x-tool_dxl1.x)/(2*initStep);
    double du1overdy = (tool_dyr1.x-tool_dyl1.x)/(2*initStep);
    double du1overdz = (tool_dzr1.x-tool_dzl1.x)/(2*initStep);
    double dv1overdx = (tool_dxr1.y-tool_dxl1.y)/(2*initStep);
    double dv1overdy = (tool_dyr1.y-tool_dyl1.y)/(2*initStep);
    double dv1overdz = (tool_dzr1.y-tool_dzl1.y)/(2*initStep);

    double du2overdx = (tool_dxr2.x-tool_dxl2.x)/(2*initStep);
    double du2overdy = (tool_dyr2.x-tool_dyl2.x)/(2*initStep);
    double du2overdz = (tool_dzr2.x-tool_dzl2.x)/(2*initStep);
    double dv2overdx = (tool_dxr2.y-tool_dxl2.y)/(2*initStep);
    double dv2overdy = (tool_dyr2.y-tool_dyl2.y)/(2*initStep);
    double dv2overdz = (tool_dzr2.y-tool_dzl2.y)/(2*initStep);

    J_flat[0]=du1overdx;
    J_flat[1]=du1overdy;
    J_flat[2]=du1overdz;
    J_flat[3]=dv1overdx;
    J_flat[4]=dv1overdy;
    J_flat[5]=dv1overdz;
    J_flat[6]=du2overdx;
    J_flat[7]=du2overdy;
    J_flat[8]=du2overdz;
    J_flat[9]=dv2overdx;
    J_flat[10]=dv2overdy;
    J_flat[11]=dv2overdz;

    visual_servo::JacobianUpdater::flat2eigen(J, J_flat);

}

void visual_servo::JacobianUpdater::initializeJacobianOri(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, std::vector<visual_servo::ToolDetector>& detector_list){
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

    // Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

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
    visual_servo::JacobianUpdater::poseMsg2Transform(transform_target, target_pose);
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);
    
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
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

    visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("update_image_J", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    // calculate J
    // define delta thetas
    double del_theta_r1, del_theta_p1, del_theta_y1, del_theta_r2, del_theta_p2, del_theta_y2;

    cv::Point del_tooltip_r_r1 = (tooltip_drl1-tool_center1)-(tooltip_drr1-tool_center1); 
    cv::Point del_tooltip_r_p1 = (tooltip_dpl1-tool_center1)-(tooltip_dpr1-tool_center1); 
    cv::Point del_tooltip_r_y1 = (tooltip_dyl1-tool_center1)-(tooltip_dyr1-tool_center1); 
    cv::Point del_tooltip_r_r2 = (tooltip_drl2-tool_center2)-(tooltip_drr2-tool_center2); 
    cv::Point del_tooltip_r_p2 = (tooltip_dpl2-tool_center2)-(tooltip_dpr2-tool_center2); 
    cv::Point del_tooltip_r_y2 = (tooltip_dyl2-tool_center2)-(tooltip_dyr2-tool_center2); 

    cv::Point del_toolframe_r_r1 = (toolframe_drl1-tool_center1)-(toolframe_drr1-tool_center1); 
    cv::Point del_toolframe_r_p1 = (toolframe_dpl1-tool_center1)-(toolframe_dpr1-tool_center1); 
    cv::Point del_toolframe_r_y1 = (toolframe_dyl1-tool_center1)-(toolframe_dyr1-tool_center1); 
    cv::Point del_toolframe_r_r2 = (toolframe_drl2-tool_center2)-(toolframe_drr2-tool_center2); 
    cv::Point del_toolframe_r_p2 = (toolframe_dpl2-tool_center2)-(toolframe_dpr2-tool_center2); 
    cv::Point del_toolframe_r_y2 = (toolframe_dyl2-tool_center2)-(toolframe_dyr2-tool_center2); 

    double del_tooltip_theta_r1 = atan2(del_tooltip_r_r1.y, del_tooltip_r_r1.x); 
    double del_tooltip_theta_p1 = atan2(del_tooltip_r_p1.y, del_tooltip_r_p1.x); 
    double del_tooltip_theta_y1 = atan2(del_tooltip_r_y1.y, del_tooltip_r_y1.y); 
    double del_tooltip_theta_r2 = atan2(del_tooltip_r_r2.y, del_tooltip_r_r2.x); 
    double del_tooltip_theta_p2 = atan2(del_tooltip_r_p2.y, del_tooltip_r_p2.x); 
    double del_tooltip_theta_y2 = atan2(del_tooltip_r_y2.y, del_tooltip_r_y2.x);

    double del_toolframe_theta_r1 = atan2(del_toolframe_r_r1.y, del_toolframe_r_r1.x); 
    double del_toolframe_theta_p1 = atan2(del_toolframe_r_p1.y, del_toolframe_r_p1.x); 
    double del_toolframe_theta_y1 = atan2(del_toolframe_r_y1.y, del_toolframe_r_y1.x); 
    double del_toolframe_theta_r2 = atan2(del_toolframe_r_r2.y, del_toolframe_r_r2.x); 
    double del_toolframe_theta_p2 = atan2(del_toolframe_r_p2.y, del_toolframe_r_p2.x); 
    double del_toolframe_theta_y2 = atan2(del_toolframe_r_y2.y, del_toolframe_r_y2.x);
    

    double du1overdr = del_tooltip_theta_r1/(2*initStepOri);
    double du1overdp = del_tooltip_theta_p1/(2*initStepOri);
    double du1overdy = del_tooltip_theta_y1/(2*initStepOri);
    double dv1overdr = del_tooltip_theta_r2/(2*initStepOri);
    double dv1overdp = del_tooltip_theta_p2/(2*initStepOri);
    double dv1overdy = del_tooltip_theta_y2/(2*initStepOri);

    double du2overdr = del_toolframe_theta_r1/(2*initStepOri);
    double du2overdp = del_toolframe_theta_p1/(2*initStepOri);
    double du2overdy = del_toolframe_theta_y1/(2*initStepOri);
    double dv2overdr = del_toolframe_theta_r2/(2*initStepOri);
    double dv2overdp = del_toolframe_theta_p2/(2*initStepOri);
    double dv2overdy = del_toolframe_theta_y2/(2*initStepOri);

    J_ori_flat[0]=du1overdr;
    J_ori_flat[1]=du1overdp;
    J_ori_flat[2]=du1overdy;
    J_ori_flat[3]=dv1overdr;
    J_ori_flat[4]=dv1overdp;
    J_ori_flat[5]=dv1overdy;
    J_ori_flat[6]=du2overdr;
    J_ori_flat[7]=du2overdp;
    J_ori_flat[8]=du2overdy;
    J_ori_flat[9]=dv2overdr;
    J_ori_flat[10]=dv2overdp;
    J_ori_flat[11]=dv2overdy;

    visual_servo::JacobianUpdater::flat2eigen(J_ori, J_ori_flat);
    std::cout << "J: " << J_ori << std::endl;
}

void visual_servo::JacobianUpdater::updateJacobian(Eigen::VectorXd& del_Pr, Eigen::VectorXd& del_r, bool ori=false){
    OptimData data;
    data.del_Pr = del_Pr;
    data.del_r = del_r;
    if (ori){
        std::vector<double> result(J_ori_flat.size());
        runLM(data, J_ori_flat, result);
        J_ori_flat = result;
        flat2eigen(J_ori, result);
    }
    else{
        std::vector<double> result(J_flat.size());
        runLM(data, J_flat, result);
        J_flat = result;
        flat2eigen(J, result);
    }
}

void visual_servo::JacobianUpdater::mainLoop(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, std::vector<visual_servo::ToolDetector> & detector_list){
    ros::Rate rate(10.0);
    visual_servo::ToolDetector detector = detector_list[0];
    initializeJacobian(cam1, cam2, detector);
    initializeJacobianOri(cam1, cam2, detector_list);

    std_msgs::Float64MultiArray Jmsg;
    cv::Point toolPos1, toolPos2;
    cv::Point tooltipPos1, tooltipPos2;
    cv::Point toolframePos1, toolframePos2;

    double roll, pitch, yaw;

    ROS_INFO("Jacobian initialized! Ready for Jacobian update.");
    // loopup current robot pose
    while (nh.ok()){
        try{
            detector.detect(cam1);
            toolPos1 = detector.getCenter();
            detector.detect(cam2);
            toolPos2 = detector.getCenter();

            listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);

            detector_list[1].detect(cam1);
            tooltipPos1 = detector_list[1].getCenter();
            detector_list[1].detect(cam2);
            tooltipPos2 = detector_list[1].getCenter();

            detector_list[2].detect(cam1);
            toolframePos1 = detector_list[2].getCenter();
            detector_list[2].detect(cam2);
            toolframePos2 = detector_list[2].getCenter();
            break;
            }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            // rate.sleep();
            }   
    }
    
    lastToolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;
    lastRobotPos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    visual_servo::JacobianUpdater::getToolRot(lastToolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    lastRobotRot << roll, pitch, yaw;

    Eigen::VectorXd encDisplFromLast;
    Eigen::VectorXd pixDisplFromLast;

    Eigen::VectorXd encAngDisplFromLast;
    Eigen::VectorXd pixAngDisplFromLast;

    while ((nh.ok())){
        while (nh.ok()){
            try{
                //*** TODO ***
                // overload detect function to detect a given image, and only capture images in this loop
                detector.detect(cam1);
                toolPos1 = detector.getCenter();
                detector.detect(cam2);
                toolPos2 = detector.getCenter();

                listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);

                detector_list[1].detect(cam1);
                tooltipPos1 = detector_list[1].getCenter();
                detector_list[1].detect(cam2);
                tooltipPos2 = detector_list[1].getCenter();

                detector_list[2].detect(cam1);
                toolframePos1 = detector_list[2].getCenter();
                detector_list[2].detect(cam2);
                toolframePos2 = detector_list[2].getCenter();
                break;
                }
            catch (tf::TransformException ex){
                // ROS_ERROR("%s",ex.what());
                }   
        }
        
        toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;
        robotPos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

        visual_servo::JacobianUpdater::getToolRot(toolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
        robotRot << roll, pitch, yaw;

        encDisplFromLast = robotPos-lastRobotPos;
        std::cout << "robotPosition: " << robotPos <<std::endl;
        std::cout << "encDisFromLast: " << encDisplFromLast << std::endl;

        pixDisplFromLast = toolPos-lastToolPos;
        std::cout << "toolPosition: " << toolPos << std::endl;
        std::cout << "pixelDisFromLast: " << pixDisplFromLast << std::endl;

        encAngDisplFromLast = robotRot-lastRobotRot;
        std::cout << "robotRotation: " << robotRot <<std::endl;
        std::cout << "encDisFromLast: " << encAngDisplFromLast << std::endl;

        pixAngDisplFromLast = toolRot-lastToolRot;
        std::cout << "toolRotation: " << toolRot << std::endl;
        std::cout << "pixelAngDisFromLast: " << pixAngDisplFromLast << std::endl;

        // std::cout << "J: " << J << std::endl;

        // std::cout << "J_ori: " << J_ori << std::endl;
        
        // update J only if greater than a distance from the last update
        if (pixDisplFromLast.norm()>= update_pix_step || encDisplFromLast.norm()>= update_enc_step){
            updateJacobian(pixDisplFromLast, encDisplFromLast);
            lastToolPos = toolPos;
            lastRobotPos = robotPos;
        } 

        // update J_ori only if greater than a distance from the last update
        if (pixAngDisplFromLast.norm()>= update_pix_ang_step || encAngDisplFromLast.norm()>= update_enc_ang_step){
            visual_servo::JacobianUpdater::limitAngDisp(pixAngDisplFromLast);
            visual_servo::JacobianUpdater::limitAngDisp(encAngDisplFromLast);
            updateJacobian(pixAngDisplFromLast, encAngDisplFromLast, true);
            lastToolRot = toolRot;
            lastRobotRot = robotRot;
        } 

        Jmsg.data = J_ori_flat;
        J_pub.publish(Jmsg);
        ros::spinOnce();
        rate.sleep();
    }
}


// #################
// ##    Utils    ##
// #################

void visual_servo::JacobianUpdater::flat2eigen(Eigen::MatrixXd& M, const double* flat){
    // check input
    int size_M = M.cols()*M.rows();

    int cur = 0;
    for (int i=0; i<M.rows(); ++i){
        for (int j=0; j<M.cols(); ++j){
            M.coeffRef(i,j) = flat[cur];
            cur++;
        }
    }
}

void visual_servo::JacobianUpdater::flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat){
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

void visual_servo::JacobianUpdater::transform2PoseMsg(tf::Transform& transform, geometry_msgs::Pose& pose){
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();

    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
}

void visual_servo::JacobianUpdater::poseMsg2Transform(tf::Transform& transform, geometry_msgs::Pose& pose){
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
}

void visual_servo::JacobianUpdater::getToolRot(Eigen::VectorXd& toolRot, cv::Point& center1, cv::Point& tooltip1, cv::Point& frametip1, cv::Point& center2, cv::Point& tooltip2, cv::Point& frametip2){
    double theta11 = atan2((tooltip1-center1).y, (tooltip1-center1).x); 
    double theta12 = atan2((frametip1-center1).y, (frametip1-center1).x); 
    double theta21 = atan2((tooltip2-center2).y, (tooltip2-center2).x); 
    double theta22 = atan2((frametip2-center2).y, (frametip2-center2).x); 
    toolRot << theta11, theta12, theta21, theta22;
}

void visual_servo::JacobianUpdater::limitAngDisp(Eigen::VectorXd& angDisp){
    for (int i=0; i<angDisp.size(); i++){
        if (angDisp(i)<0.09){
            angDisp(i) = 0.0;
        }
    }
}