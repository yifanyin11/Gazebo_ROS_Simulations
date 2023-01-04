#include "Jacobian_updater.hpp"

visual_servo::JacobianUpdater::JacobianUpdater(ros::NodeHandle& nh, std::string& toolPos_topic):
nh(nh){
    // ros initialization
    ros::Rate rate(10);
    
    // initializations
    J_topic = "/visual_servo/tooltip_positions";
    dof_robot = 3;
    num_features = 4;

    update_pix_step = 100;
    update_enc_step = 5000;
    servoMaxStep = 100;
    initStep = 1000;

    toolPosReceived = false;
    encReceived = false;

    toolPosition.resize(num_features);
    lastToolPos.resize(num_features);
    robotPosition.resize(dof_robot);
    lastRobotPos.resize(dof_robot);
    J.resize(num_features, dof_robot);
    J_flat.resize(dof_robot*num_features);

    // publishers
    J_pub = nh.advertise<std_msgs::Float64MultiArray>(J_topic, 1);
}

static void visual_servo::JacobianUpdater::evalCostFunction(const double *params, int num_inputs, const void *inputs, double *fvec, int *info)
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

static bool visual_servo::JacobianUpdater::runLM(const OptimData& optim_data, const std::vector<double>& initial_state, std::vector<double>& result)
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

int visual_servo::JacobianUpdater::initializeJacobian(visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, visual_servo::ToolDetector& detector, int initStep=0.05){
    // Ros setups
    ros::AsyncSpinner spinner(1);
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

    // define points
    cv::Point tool_center, tool_dxl, tool_dyl, tool_dzl, tool_dxr, tool_dyr, tool_dzr;
    // get current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    // define target pose and initialize it as current pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position = current_pose.pose.position;
    // move to x-
    target_pose.position.x = target_pose.position.x-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at x-
    detector.detect(cam);
    tool_dxl = detector.getToolCenter();

    // move to x+
    target_pose.position.x = target_pose.position.x+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at x+
    detector.detect(cam);
    tool_dxr = detector.getToolCenter();
    
    // move to y-
    target_pose.position.x = target_pose.position.x-initStep;
    target_pose.position.y = target_pose.position.y-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at y-
    detector.detect(cam);
    tool_dyl = detector.getToolCenter();

    // move to y+
    target_pose.position.y = target_pose.position.y+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at x+
    detector.detect(cam);
    tool_dyr = detector.getToolCenter();

    // move to z-
    target_pose.position.y = target_pose.position.y-initStep;
    target_pose.position.z = target_pose.position.z-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at z-
    detector.detect(cam);
    tool_dzl = detector.getToolCenter();

    // move to z+
    target_pose.position.z = target_pose.position.z+2*initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // detect tool image position at z+
    detector.detect(cam);
    tool_dzr = detector.getToolCenter();

    // go back to center
    target_pose.position.z = target_pose.position.z-initStep;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_arm.move();

    // calculate J
    du1overdx = (tool_dxr-tool_dxl)/dx;
    du1overdy = (tool_dyr-center_overhead[0])/dy;
    du1overdz = (posz_overhead[0]-center_overhead[0])/dz;
    dv1overdx = (posx_overhead[1]-center_overhead[1])/dx;
    dv1overdy = (posy_overhead[1]-center_overhead[1])/dy;
    dv1overdz = (posz_overhead[1]-center_overhead[1])/dz;

    du2overdx = (posx_sideview[0]-center_sideview[0])/dx;
    du2overdy = (posy_sideview[0]-center_sideview[0])/dy;
    du2overdz = (posz_sideview[0]-center_sideview[0])/dz;
    dv2overdx = (posx_sideview[1]-center_sideview[1])/dx;
    dv2overdy = (posy_sideview[1]-center_sideview[1])/dy;
    dv2overdz = (posz_sideview[1]-center_sideview[1])/dz;

    J = [du1overdx, du1overdy, du1overdz, dv1overdx, dv1overdy, dv1overdz, du2overdx, du2overdy, du2overdz, dv2overdx, dv2overdy, dv2overdz];

    return 0;
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

void visual_servo::JacobianUpdater::mainLoop(visual_servo::ToolDetector& detector){

    if (initializeJacobian(nh)==-1){
        return -1;
    }

    std_msgs::Float64MultiArray Jmsg;

    ROS_INFO("Jacobian initialized! Ready for Jacobian update.");

    Eigen::VectorXd robotPos;
    Eigen::VectorXd toolPos;
    Eigen::VectorXd encDisplFromLast;
    Eigen::VectorXd pixDisplFromLast;

    while ((nh.ok())){
        
        if (encReceived){
            robotPos = robotPosition;
            encDisplFromLast = robotPos-lastRobotPos;
            std::cout << "robotPosition: " << robotPosition <<std::endl;
            std::cout << "encDisFromLast: " << encDisplFromLast << std::endl;
        }
        if (toolPosReceived){
            toolPos = toolPosition;
            pixDisplFromLast = toolPos-lastToolPos;
            std::cout << "toolPosition: " << toolPos << std::endl;
            std::cout << "pixelDisFromLast: " << pixDisplFromLast << std::endl;
        }

        std::cout << "J: " << J << std::endl;
        
        // update J only if greater than a distance from the last update
        if (encReceived && toolPosReceived && pixDisplFromLast.norm()>= update_pix_step && encDisplFromLast.norm()>= update_enc_step){
            updateJacobian(pixDisplFromLast, encDisplFromLast);
            lastToolPos = toolPos;
            lastRobotPos = robotPos;
        } 

        Jmsg.data = J_flat;
        Jacobian_publisher.publish(Jmsg);
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
