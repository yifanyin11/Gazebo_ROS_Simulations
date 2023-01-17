#include "visual_servo_controller.hpp"

// ####################
// # public functions #
// ####################

// constructors
visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, bool target_topic=false, double tol=5, double tol_ori=0.16) :
    nh(nh){

    tolerance = tol;
    tolerance_ori = tol_ori;

    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 0.2;
    servoAngMaxStep = 0.25;
    K = 0.3;
    K_ori = 0.1;
    constJTh = 10;
    constJTh_ori = 0.16;

    JChecked = false;
    continueLoop = true;
    targetReceived = false;
    closeUpdateJOri = false;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/Jacobian", 1, &VisualServoController::JacobianCallback, this);
    if (target_topic){
        target_sub = nh.subscribe("/visual_servo/goal", 100, &VisualServoController::targetCallback, this);
    }
    // containers initialization
    targets.resize(num_features);
    toolPos.resize(num_features);
    toolRot.resize(num_features);
    J.resize(num_features, dof);
    J_ori.resize(num_features, dof);
    controlError.resize(num_features);
    controlError(0)=DBL_MAX; // set norm of control error to max
    controlError_ori.resize(num_features);
    controlError_ori(0)=DBL_MAX; // set norm of control error to max
}

visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, Eigen::VectorXd& targets_, double tol=5, double tol_ori=0.16) :
    nh(nh){

    tolerance = tol;
    tolerance_ori = tol_ori;
    targets = targets_;

    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 0.2;
    servoAngMaxStep = 0.25;
    K = 0.3;
    K_ori = 0.1;
    constJTh = 10;
    constJTh_ori = 0.16;

    JChecked = false;
    continueLoop = true;
    targetReceived = true;
    closeUpdateJOri = false;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/image_Jacobian", 1, &VisualServoController::JacobianCallback, this);
    // containers initialization
    toolPos.resize(num_features);
    toolRot.resize(num_features);
    J.resize(num_features, dof);
    J_ori.resize(num_features, dof);
    controlError.resize(num_features);
    controlError(0)=DBL_MAX; // set norm of control error to max
    controlError_ori.resize(num_features);
    controlError_ori(0)=DBL_MAX; // set norm of control error to max
}

void visual_servo::VisualServoController::JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // not updating J when control error is small
    if (msg->data.size()==dof*num_features){
        if (controlError.norm()>constJTh){
        J_flat = msg->data;
        flat2eigen(J, J_flat);
        }
    }
    else{
        std::vector<double> temp1(msg->data.begin(), msg->data.begin()+dof*num_features);
        J_flat = temp1;
        flat2eigen(J, J_flat);
        std::vector<double> temp2(msg->data.begin()+dof*num_features, msg->data.end());
        J_ori_flat = temp2;
        flat2eigen(J_ori, J_ori_flat);
    }
    // else{
    //     std::vector<double> temp1(msg->data.begin(), msg->data.begin()+dof*num_features);
    //     J_flat = temp1;
    //     flat2eigen(J, J_flat);
    //     if (controlError.norm()<constJTh && controlError_ori.norm()<constJTh_ori){
    //         closeUpdateJOri = true;
    //     }
    //     else{
    //         if (!closeUpdateJOri){
    //             std::vector<double> temp2(msg->data.begin()+dof*num_features, msg->data.end());
    //             J_ori_flat = temp2;
    //             flat2eigen(J_ori, J_ori_flat);
    //         }
    //     }
    // }

    if (!JChecked){
        JChecked = true;
    }
}

void visual_servo::VisualServoController::targetCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (!targetReceived){
        targetReceived = true;
    } 
    targets << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
}

void visual_servo::VisualServoController::setServoMaxStep(int step){
    servoMaxStep = step;
}

void visual_servo::VisualServoController::setK(double K_){
    K = K_;
}

void visual_servo::VisualServoController::setFreq(int f){
    freq = f;
}

bool visual_servo::VisualServoController::stopSign(){
    return !continueLoop;
}

Eigen::VectorXd visual_servo::VisualServoController::getToolPosition(){
    return toolPos;
}

// update member variables
void visual_servo::VisualServoController::directionIncrement(Eigen::VectorXd& increment, visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, 
visual_servo::ToolDetector& detector){
    ros::Rate loopRate(freq);

    while(nh.ok()){
        ros::spinOnce();
        break;
    }

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }
    if (targets.size()>num_features){
        ROS_ERROR("ERROR! Target features are more than needed for the position servo. Check target input or switch to full pose servo.");
    }
    if (targets.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }
    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ------");
        while((nh.ok()) && !JChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }

    detector.detect(cam1);
    cv::Point toolPos1 = detector.getCenter();
    detector.detect(cam2);
    cv::Point toolPos2 = detector.getCenter();

    toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;

    controlError = targets-toolPos;

    if (controlError.norm()<tolerance){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }
    
    std::cout << "target: " << targets << std::endl;
    std::cout << "toolPos: " << toolPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    Eigen::MatrixXd J_pinv = (J.transpose()*J).inverse()*J.transpose();
    increment = K*J_pinv*controlError;
    std::cout << "increment" << increment << std::endl;

    if (increment.norm()>servoMaxStep){
        limInc(increment, servoMaxStep);
    }
    else{
        ROS_INFO("Refining ---");
    }

    std::cout << "increment after limit" << increment << std::endl;
}

void visual_servo::VisualServoController::poseDirectionIncrement(Eigen::VectorXd& increment, visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, 
std::vector<visual_servo::ToolDetector>& detector_list){

    ros::Rate loopRate(freq);

    while(nh.ok()){
        ros::spinOnce();
        break;
    }

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }

    if (targets.size()!=num_features*2){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", 2*num_features);
    }

    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ------");
        while((nh.ok()) && !JChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }

    Eigen::VectorXd increment_pos, increment_ori;

    Eigen::VectorXd target_pos = targets.head(4);
    Eigen::VectorXd target_ori = targets.tail(4);

    std::cout << "targets divided!" << std::endl;

    cv::Point toolPos1, toolPos2;
    cv::Point tooltipPos1, tooltipPos2;
    cv::Point toolframePos1, toolframePos2;

    detector_list[0].detect(cam1);
    toolPos1 = detector_list[0].getCenter();
    detector_list[0].detect(cam2);
    toolPos2 = detector_list[0].getCenter();

    std::cout << "target detected!" << std::endl;

    toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;

    std::cout << "toolPos assigned!" << std::endl;

    controlError = target_pos-toolPos;

    std::cout << "control error assigned!" << std::endl;

    detector_list[1].detect(cam1);
    tooltipPos1 = detector_list[1].getCenter();
    detector_list[1].detect(cam2);
    tooltipPos2 = detector_list[1].getCenter();

    detector_list[2].detect(cam1);
    toolframePos1 = detector_list[2].getCenter();
    detector_list[2].detect(cam2);
    toolframePos2 = detector_list[2].getCenter();

    visual_servo::VisualServoController::getToolRot(toolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);

    std::cout << "got tool rot!" << std::endl;

    controlError_ori = target_ori-toolRot;

    // *********************
    // visual_servo::VisualServoController::stdAngControlError(controlError_ori);


    if (controlError.norm()<tolerance && controlError_ori.norm()<tolerance_ori){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }
    
    std::cout << "target_pos: " << target_pos << std::endl;
    std::cout << "toolPos: " << toolPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    Eigen::MatrixXd J_pinv = (J.transpose()*J).inverse()*J.transpose();
    increment_pos = K*J_pinv*controlError;

    std::cout << "increment_pos: " << increment_pos << std::endl;

    std::cout << "**************" << increment_pos << std::endl;

    std::cout << "target_ori: " << target_ori << std::endl;
    std::cout << "toolRot: " << toolRot << std::endl;
    std::cout << "control_error_ori: " << controlError_ori << std::endl;

    Eigen::MatrixXd J_ori_pinv = (J_ori.transpose()*J_ori).inverse()*J_ori.transpose();
    increment_ori = K_ori*J_ori_pinv*controlError_ori;

    std::cout << "increment_ori: " << increment_ori << std::endl;

    if (increment_pos.norm()>servoMaxStep){
        limInc(increment_pos, servoMaxStep);
    }
    else{
        ROS_INFO("Refining position ---");
    }

    if (increment_ori.norm()>servoAngMaxStep){
        limInc(increment_ori, servoAngMaxStep);
    }
    else{
        ROS_INFO("Refining orientation ---");
    }

    increment << increment_pos, increment_ori;

    std::cout << "full increment: " << increment << std::endl;
    // std::cout << "increment after limit" << increment << std::endl;
}


// #################
// ##    Utils    ##
// #################

void visual_servo::VisualServoController::flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat){
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

void visual_servo::VisualServoController::limInc(Eigen::VectorXd& v, double stepSize){
    double r;
    // a vector with abs of v
    Eigen::VectorXd abs_v(v.size());
    for (int i=0; i<abs_v.size(); ++i){
        abs_v(i) = abs(v(i));
    }
    // initializations
    double max_abs=abs_v(0);
    // find the maximum absolute value
    for (int i=0; i<abs_v.size(); ++i){
        if (abs_v(i)>max_abs){
            max_abs = abs_v(i);
        }
    }
    
    if (max_abs > stepSize){
        std::cout << "Enter limiting velocity!" << std::endl;
        r = stepSize*1.0/max_abs;
        std::cout << "r: " << r << std::endl;
        v = r*v;
    }
}

void visual_servo::VisualServoController::getToolRot(Eigen::VectorXd& toolRot, cv::Point& center1, cv::Point& tooltip1, cv::Point& frametip1, cv::Point& center2, cv::Point& tooltip2, cv::Point& frametip2){
    double theta11 = atan2((tooltip1-center1).y, (tooltip1-center1).x); 
    double theta12 = atan2((frametip1-center1).y, (frametip1-center1).x); 
    double theta21 = atan2((tooltip2-center2).y, (tooltip2-center2).x); 
    double theta22 = atan2((frametip2-center2).y, (frametip2-center2).x); 
    toolRot << theta11, theta12, theta21, theta22;
}

void visual_servo::VisualServoController::stdAngControlError(Eigen::VectorXd& controlError_ori){
    for (int i=0; i<controlError_ori.size(); i++){
        if (abs(controlError_ori(i))>M_PI){
            controlError_ori(i) = (signbit(controlError_ori(i)) ? 1.0 : -1.0)*(2*M_PI-abs(controlError_ori(i)));
        }
    }
}