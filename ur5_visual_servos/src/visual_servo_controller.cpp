#include "visual_servo_controller.hpp"

// ####################
// # public functions #
// ####################

// constructors
visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, double& tol) :
    nh(nh), tolerance(tol){

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    // spinner.start();
    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 100;
    K = 0.03;
    JChecked = false;
    continueLoop = true;
    targetReceived = false;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/Jacobian", 1, &VisualServoController::JacobianCallback, this);
    target_sub = nh.subscribe("/visual_servo/goal", 100, &VisualServoController::targetCallback, this);
    // containers initialization
    targets.resize(num_features);
    toolPosition.resize(num_features);
    robotPosition.resize(dof);
    J.resize(num_features, dof);
    controlError.resize(num_features);
    controlError.setZero();
}

visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, double& tol, Eigen::VectorXd& targets) :
    nh(nh), tolerance(tol), targets(targets){

    // ros::AsyncSpinner spinner(3); // Use 3 threads
    // spinner.start();
    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 100;
    K = 0.03;
    JChecked = false;
    continueLoop = true;
    targetReceived = true;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/Jacobian", 1, &VisualServoController::JacobianCallback, this);
    toolPos_sub = nh.subscribe("/visual_servo/tool_positions", 1, &VisualServoController::controlInputsCallback, this);
    // containers initialization
    targets.resize(num_features);
    toolPosition.resize(num_features);
    robotPosition.resize(dof);
    J.resize(num_features, dof);
    controlError.resize(num_features);
    controlError.setZero();
}

void visual_servo::VisualServoController::JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    J_flat = msg->data;
    flat2eigen(J, J_flat);
    if (!JChecked){
        dof = J.cols();
        num_features = J.rows();
        JChecked = true;
    }
}

void visual_servo::VisualServoController::targetCallback(const VisualServoGoal::ConstPtr& msg){
    if (!targetReceived){
        targetReceived = true;
        targets << msg->camera1_x, msg->camera1_y, msg->camera2_x, msg->camera2_y;
    } 
}

void visual_servo::VisualServoController::controlInputsCallback(const VisualServoPositions::ConstPtr& msg){
    std_msgs::Bool stopmsg;
    if (msg->camera1_x>0 && msg->camera2_x>0){
        tipPosition << msg->camera1_x, msg->camera1_y, msg->camera2_x, msg->camera2_y;
        robotPosition << msg->encoder_x, msg->encoder_y, msg->encoder_z;

        if (controlError.isZero(0) || controlError.norm()>tolerance){
            continueLoop = true;
        }
        else{
            continueLoop = false;
        }
    }
}

void visual_servo::VisualServoController::setServoMaxStep(int step){
    servoMaxStep = step;
}

void visual_servo::VisualServoController::setUpdatePixStep(int step){
    update_pix_step = step;
}

void visual_servo::VisualServoController::setUpdateEncStep(int step){
    update_enc_step = step;
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
    return tipPosition;
}

// update member variables
void visual_servo::VisualServoController::directionIncrement(Eigen::VectorXd& increment, visual_servo::ToolDetector& detector){
    ros::Rate loopRate(freq);
    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }
    if (targets.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }
    if (tipPosition.size()!=num_features){
        ROS_ERROR("Tip position size inconsistent! Should match: %d", num_features);
    }

    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ------");
        while((nh.ok()) && !JChecked){
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }

    Eigen::VectorXd tipPos = tipPosition;
    controlError = targets-tipPos;
    Eigen::VectorXd robotPos = robotPosition;
    
    std::cout << "target: " << targets << std::endl;
    std::cout << "tipPosition: " << tipPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    Eigen::VectorXd J_pinv = (J.transpose()*J).inverse()*J.transpose();
    increment = K*J_pinv*controlError;
    std::cout << "increment" << increment << std::endl;

    if (increment.norm()>servoMaxStep*20){
        limtVec(increment, servoMaxStep*50);
    }
    else{
        ROS_INFO("Refining ---");
    }
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

void visual_servo::VisualServoController::limtVec(Eigen::VectorXd& v, int stepSize){
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
        r = stepSize/max_abs;
        v = r*v;
    }
}

