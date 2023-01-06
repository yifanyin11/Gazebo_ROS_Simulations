#include "Jacobian_updater.hpp"

visual_servo::JacobianUpdater::JacobianUpdater(ros::NodeHandle& nh, std::string& J_topic_):
nh(nh){
    
    // initializations
    J_topic = J_topic_;
    dof_robot = 3;
    num_features = 4;

    update_pix_step = 50;
    update_enc_step = 0.3;
    initStep = 0.05;

    toolPos.resize(num_features);
    lastToolPos.resize(num_features);
    robotPos.resize(dof_robot);
    lastRobotPos.resize(dof_robot);
    J.resize(num_features, dof_robot);
    J_flat.resize(dof_robot*num_features);

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
    cv::Point tool_center; 
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
    ros::Duration(1.0).sleep();

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
    ros::Duration(1.0).sleep();

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
    ros::Duration(1.0).sleep();

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
    ros::Duration(1.0).sleep();

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
    ros::Duration(1.0).sleep();

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
    ros::Duration(1.0).sleep();

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

    ros::Duration(1.0).sleep();

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

void visual_servo::JacobianUpdater::updateJacobian(Eigen::VectorXd& del_Pr, Eigen::VectorXd& del_r){
    OptimData data;
    data.del_Pr = del_Pr;
    data.del_r = del_r;
    int count = 0;
    std::vector<double> result(J_flat.size());
    runLM(data, J_flat, result);
    J_flat = result;
    flat2eigen(J, result);
}

void visual_servo::JacobianUpdater::mainLoop(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, visual_servo::ToolDetector& detector){
    ros::Rate rate(10.0);
    initializeJacobian(cam1, cam2, detector);

    std_msgs::Float64MultiArray Jmsg;
    cv::Point toolPos1, toolPos2;

    ROS_INFO("Jacobian initialized! Ready for Jacobian update.");
    // loopup current robot pose
    while (nh.ok()){
        try{
            listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);
            detector.detect(cam1);
            toolPos1 = detector.getCenter();
            detector.detect(cam2);
            toolPos2 = detector.getCenter();
            break;
            }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            // rate.sleep();
            }   
    }
    
    lastToolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;
    lastRobotPos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    Eigen::VectorXd encDisplFromLast;
    Eigen::VectorXd pixDisplFromLast;

    while ((nh.ok())){
        while (nh.ok()){
            try{
                //*** TODO ***
                // overload detect function to detect a given image, and only capture images in this loop
                listener.lookupTransform("/world", "/ee_link",  ros::Time(0), transform);
                detector.detect(cam1);
                toolPos1 = detector.getCenter();
                detector.detect(cam2);
                toolPos2 = detector.getCenter();
                break;
                }
            catch (tf::TransformException ex){
                // ROS_ERROR("%s",ex.what());
                }   
        }
        
        toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;
        robotPos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

        encDisplFromLast = robotPos-lastRobotPos;
        std::cout << "robotPosition: " << robotPos <<std::endl;
        std::cout << "encDisFromLast: " << encDisplFromLast << std::endl;

        pixDisplFromLast = toolPos-lastToolPos;
        std::cout << "toolPosition: " << toolPos << std::endl;
        std::cout << "pixelDisFromLast: " << pixDisplFromLast << std::endl;

        std::cout << "J: " << J << std::endl;
        
        // update J only if greater than a distance from the last update
        if (pixDisplFromLast.norm()>= update_pix_step || encDisplFromLast.norm()>= update_enc_step){
            updateJacobian(pixDisplFromLast, encDisplFromLast);
            lastToolPos = toolPos;
            lastRobotPos = robotPos;
        } 

        Jmsg.data = J_flat;
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
