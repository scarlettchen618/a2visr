#include"a2VISR.h"

#define GRAVITATIONAL_ACC 9.794 // The gravity of Shanghai

/**
 * @brief Construct a new dwe::dwe object
 */
dwe::dwe(ros::NodeHandle& nh) {

    ROS_INFO("**********START ROS NODE**********");
    // br_uav_to_origin = br;

    //============================== Read ros parameter ===========================//
    const std::string& node_name = ros::this_node::getName();
    nh.getParam(node_name + "/" + "dataType", dataType_);

    // param for MAP sliding window
    nh.getParam(node_name + "/" + "mapwin/delta", delta_);
    nh.getParam(node_name + "/" + "mapwin/lopt",  lopt_);
    nh.getParam(node_name + "/" + "mapwin/order", order_);
    nh.getParam(node_name + "/" + "mapwin/k", k_);
    nh.getParam(node_name + "/" + "mapwin/rate", rate_);
    nh.getParam(node_name + "/" + "mapwin/init_num", init_num_);
    nh.getParam(node_name + "/" + "mapwin/vicon_flag", vicon_flag_);
    nh.getParam(node_name + "/" + "mapwin/param_flag", param_flag_);

    // param for dwe_param
    nh.getParam(node_name + "/" + "dwe_param/paramPp",  paramPp_);
    nh.getParam(node_name + "/" + "dwe_param/paramPv",  paramPv_);
    nh.getParam(node_name + "/" + "dwe_param/paramQp",  sensor.param.imu_p[0]);
    nh.getParam(node_name + "/" + "dwe_param/paramQv",  sensor.param.imu_v[0]);
    nh.getParam(node_name + "/" + "dwe_param/paramR",  sensor.param.uwb);
    nh.getParam(node_name + "/" + "dwe_param/paramU",  sensor.param.flow[0]);
    nh.getParam(node_name + "/" + "dwe_param/paramUz", sensor.param.hgt);
    nh.getParam(node_name + "/" + "dwe_param/paramC",  sensor.param.cam[0]);
    nh.getParam(node_name + "/" + "dwe_param/paramCz", sensor.param.cam[2]);
    sensor.param.imu_p[1] = sensor.param.imu_p[0];
    sensor.param.imu_p[2] = sensor.param.imu_p[0];
    sensor.param.imu_v[1] = sensor.param.imu_v[0];
    sensor.param.imu_v[2] = sensor.param.imu_v[0];

    sensor.param.flow[1] = sensor.param.flow[0];
    sensor.param.flow[2] = sensor.param.flow[0] * sensor.param.flow[0] / sensor.param.hgt;

    sensor.param.cam[1] = sensor.param.cam[0];

    vector<double> kappa_vec(3);
    nh.getParam(node_name + "/" + "dwe_param/kappa", kappa_vec);
    for (int i = 0; i < 3; i++) {
        kappa_(i) = kappa_vec[i];
    }
    MHE_kappa_(0) = kappa_(0);
    MHE_kappa_(1) = kappa_(1);
    MHE_kappa_(2) = kappa_(2);
    // param for msg_sensor
    nh.getParam(node_name + "/" + "msg_sensor/uwbnum", uwbnum_);
    nh.getParam(node_name + "/" + "msg_sensor/camnum", camnum_);

    // param for msg_bias
    nh.getParam(node_name + "/" + "msg_bias/bias_d", bias_d_);

    bias_cam_ = Eigen::MatrixXd(3, camnum_);
    vector<double> bias_cam_vec(3*camnum_);
    nh.getParam(node_name + "/" + "msg_bias/bias_cam", bias_cam_vec);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < camnum_; j++) {
            bias_cam_(i,j) = bias_cam_vec[camnum_ * i + j];
        }
    }

    vector<double> bias_marker_vec(3);
    vector<double> bias_marker_ugv_vec(3);
    nh.getParam(node_name + "/" + "msg_bias/bias_marker", bias_marker_vec);
    nh.getParam(node_name + "/" + "msg_bias/bias_marker_ugv", bias_marker_ugv_vec);
    for (int i = 0; i < 3; i++) {
        bias_marker_(i) = bias_marker_vec[i];
        bias_marker_ugv_(i) = bias_marker_ugv_vec[i];
    }

    // pos_uav_init_vec [mark]  to [MOCAP] 
    // pos_uav_init_    [UAV] to [MOCAP] 
    vector<double> pos_uav_init_vec(3);
    nh.getParam(node_name + "/" + "msg_sensor/pos_uav_init", pos_uav_init_vec);
    for (int i = 0; i < 3; i++) {
        pos_uav_init_(i) = pos_uav_init_vec[i] + bias_marker_(i);
    }

    // pos_uav2uwb01_init_ [UAV] to [UWB0]
    // pos_uwb01_init_     [UWB0] to [MOCAP]
    vector<double> pos_uav2uwb01_init_vec(3);
    nh.getParam(node_name + "/" + "msg_sensor/pos_uav2uwb01_init", 
                                            pos_uav2uwb01_init_vec);
    for (int i = 0; i < 3; i++) {
        pos_uav2uwb01_init_(i) = pos_uav2uwb01_init_vec[i];
        pos_uwb01_init_(i) = pos_uav_init_(i) - pos_uav2uwb01_init_(i);
    }

    // param for msg_bias
    nh.getParam(node_name + "/" + "msg_bias/bias_imu", bias_imu_);

    vector<double> bias_uwb_vec(3);
    nh.getParam(node_name + "/" + "msg_bias/bias_uwb", bias_uwb_vec);
    for (int j = 0; j < 3; j++) {
        bias_uwb_(j) = bias_uwb_vec[j];
    }

    vector<double> bias_pos_vec(3);
    nh.getParam(node_name + "/" + "msg_bias/bias_pos", bias_pos_vec);
    for (int i = 0; i < 3; i++) {
        bias_pos_(i) = bias_pos_vec[i];
    }

    //========================== Initialize matrix param ===========================//
    // window size
    LOOP_num_ = 1;
    num_state_= k_*lopt_; 
    count_j_ = num_state_ + delta_;
    startNum_ = init_num_*num_state_;

    paramX0_  = Eigen::MatrixXd::Zero(6, num_state_+1);
    paramIMU_ = Eigen::MatrixXd::Zero(6, num_state_);
    paramX0_.topRows(3).setConstant(paramPp_);
    paramX0_.bottomRows(3).setConstant(paramPv_);
    paramIMU_.topRows(3).setConstant(sensor.param.imu_p[0]);
    paramIMU_.bottomRows(3).setConstant(sensor.param.imu_v[0]);

    paramUWB_ = Eigen::MatrixXd::Constant(uwbnum_, num_state_+1, sensor.param.uwb);
    paramCAM_  = Eigen::MatrixXd::Zero(camnum_*3, num_state_+1);
    paramCAM_.topRows(2).setConstant(sensor.param.cam[0]);
    paramCAM_.bottomRows(1).setConstant(sensor.param.cam[2]);

    paramFLOW_  = Eigen::MatrixXd::Zero(3, num_state_+1);
    paramFLOW_.topRows(2).setConstant(sensor.param.flow[0]);
    paramFLOW_.bottomRows(1).setConstant(sensor.param.flow[2]);

    paramHGT_ = Eigen::MatrixXd::Constant(1, num_state_+1, sensor.param.hgt);

    // Calculate the scaling factor
    double flow_scale = 260.0 / 255.0;
    sensor.param.sum = sensor.param.imu_p.sum() + sensor.param.imu_v.sum()
                      + sensor.param.uwb
                      + flow_scale * (sensor.param.flow.sum() + sensor.param.hgt)
                      + sensor.param.cam.sum();
 
    auto normalize = [&](double v) { return v / sensor.param.sum; };
    sensor.factor.uwb = normalize(sensor.param.uwb);
    sensor.factor.hgt = normalize(flow_scale * sensor.param.hgt);
    sensor.factor.sum = sensor.param.sum;
    for (int i = 0; i<3; ++i){
        sensor.factor.imu_p[i] = normalize(sensor.param.imu_p[i]);
        sensor.factor.imu_v[i] = normalize(sensor.param.imu_v[i]);
        sensor.factor.cam[i] = normalize(sensor.param.cam[i]);
        sensor.factor.flow[i] = normalize(flow_scale * sensor.param.flow[i]);
    }

    cam_cbs_.resize(camnum_);
    MHE_imu_.setZero(3,num_state_+1);
    MHE_imu_att_.setZero(4,num_state_+1);
    MHE_imu_residual_.setZero(3,num_state_+1);

    MHE_uwb_.setZero(uwbnum_,num_state_+1);
    MHE_uwb_residual_.setZero(uwbnum_,k_*num_state_+1);

    MHE_uwb_bias_.setZero(3,num_state_+1);
    MHE_uwb_bias_.colwise() = bias_uwb_.row(0).transpose();

    MHE_cam_.setZero(3*camnum_,num_state_+1);
    MHE_cam_residual_.setZero(3*camnum_,k_*num_state_+1);

    MHE_flow_.setZero(3,k_*num_state_+1);
    MHE_flow_residual_.setZero(3,k_*num_state_+1);
    MHE_reletive_vel_.setZero(3,num_state_+1);
    
    MHE_height_.setZero(1,num_state_+1);
    MHE_hgt_residual_.setZero(1,k_*num_state_+1);
    MHE_reletive_height_.setZero(1,num_state_+1);

    deltaTime_.setZero(1,num_state_+1);

    xt_.setZero(6,num_state_+1);
    xt_vel_.setZero(3,num_state_+1);
    xt_real_.setZero(6,num_state_+1);
    x1_.setZero(6,num_state_+1);
    xt_real_f_.setZero(6,num_state_+1);
    xt_f_.setZero(6,num_state_+1);

    VICON_PosUAV_.setZero(3,num_state_+1);
    VICON_PosUWB_.setZero(3*uwbnum_,num_state_+1);
    VICON_VelUAV_.setZero(3,num_state_+1);
    VICON_VelUWB_.setZero(3,num_state_+1);
    
    VICON_uwb_.setZero(uwbnum_,num_state_+1); 
    VICON_uwb_ini.setZero(3,uwbnum_); 
    VICON_PosUAV2UWB_.setZero(6,1);
    VICON_PosUAV2_fixedorigin_.setZero(3,1);

    current_cam_stamp.setZero(1,camnum_); 
    last_cam_stamp.setZero(1,camnum_); 
    last_update_time.setZero(1,camnum_); 

    MHE_ugv2origin_.setZero(3,num_state_+1);
    xt_2origin_.setZero(3,num_state_+1);

    // xpre(6*6),u(6*6),r(1*1*num),cam_pos(3*3*num),uwb_pos(3*3)
    Ex.setZero((6+6+uwbnum_+camnum_*3+3+1)*(num_state_+1), 6*(num_state_+1));
    // xpre(6*6),u(6*6),r(1*1*num),cam_pos(3*3*num),uwb_pos(3*3)
    Eu.setZero((6+6+uwbnum_+camnum_*3+3+1)*(num_state_+1), 
               (6+3+uwbnum_+camnum_*3+3+1)*(num_state_+1));
    // xpre(6*6),u(6*6),r(1*1*num),cam_pos(3*3*num),uwb_pos(3*3)
    W.setZero((6+6+uwbnum_+camnum_*3+3+1)*(num_state_+1), 
              (6+6+uwbnum_+camnum_*3+3+1)*(num_state_+1));
    // xpre(6*1),u(3*1),r(1*num),cam_pos(3*num),uwb_pos(3*1)
    ur.setZero((6+3+uwbnum_+3*camnum_+3+1)*(num_state_+1),1);
    xxe.setZero(6,(num_state_+1));

    t_matrix.setZero(6*(num_state_+1),6*(order_+1));
    for (int j=1;j<=6;j++){
        t_matrix(j-1,(j-1)*(order_+1)) = 1;
    }
}

dwe::~dwe(){
}

/**
 * @brief main cb
 */
void dwe::imu_new_cb(const sensor_msgs::Imu::ConstPtr &msg){
    imu_msg = *msg;
}

/**
 * @brief initial data xt_ with vicon
 */
void dwe::Initialdataxt(){
    ROS_INFO("---------- Initialize with vicon ----------");
    // initialize loc with vicon, use UWB01 as initial point
    xt_.block<3,1>(0, num_state_) = VICON_PosUAV_.col(num_state_) - 
                                    VICON_PosUWB_.col(num_state_);
    xt_.block<3,1>(3, num_state_) = VICON_VelUAV_.col(num_state_) - 
                                    VICON_VelUWB_.col(num_state_);

    // initialize loc with vicon
    xt_real_.block<3,1>(0, num_state_) = VICON_PosUAV_.col(num_state_);
    xt_real_.block<3,1>(3, num_state_) = VICON_VelUAV_.col(num_state_);

    xt_real_f_.rightCols(1) = xt_real_.rightCols(1);
    xt_f_.rightCols(1) = xt_.rightCols(1);
    xt_2origin_.rightCols(1) = xt_.topRows(3).rightCols(1);

    // Initialize velocity with VICON
    xt_vel_.block<3,1>(0, num_state_) = VICON_VelUAV_.col(num_state_) - 
                                        VICON_VelUWB_.col(num_state_);

    // get ini vicon uwb pos to avoid the loss of vicon
    VICON_uwb_ini.col(0) = VICON_PosUWB_.block(0,num_state_,3,1);

} 
/**
 * @brief initial data xt_ with pre-get param
 */

void dwe::InitialWithParam(){
    ROS_INFO("---------- Initialize with param----------");
    // xt_ [UAV] to [UWB0] 
    xt_.block<3,1>(0, num_state_) = pos_uav2uwb01_init_;
    xt_.block<3,1>(3, num_state_).setZero();

    // xt_real_ [UAV] to [MOCAP] 下的位置
    xt_real_.block<3,1>(0, num_state_) = pos_uav_init_;
    xt_real_.block<3,1>(3, num_state_).setZero();
    
    xt_real_f_.rightCols(1) = xt_real_.rightCols(1);
    xt_f_.rightCols(1) = xt_.rightCols(1);
    xt_2origin_.rightCols(1) = xt_.topRows(3).rightCols(1);
    xt_vel_.col(num_state_).setZero();

    // get ini vicon uwb pos to avoid the loss of vicon
    VICON_uwb_ini.col(0) = pos_uwb01_init_;
} 


/**
 * @brief init
 */
void dwe::initializeEstimation() {
    if (LOOP_num_ > count_j_ + 1 && LOOP_num_ == startNum_) {
        hasStartEst_ = true;
    }

    if (!hasStartEst_) {
        param_flag_ ? Initialdataxt() : InitialWithParam();
        ROS_INFO("Initialized data: x:%f y:%f z:%f", xt_(0, num_state_), 
                xt_(1, num_state_), xt_(2, num_state_));
        ROS_INFO("dt: %.4lf seconds", dt_);
    }
}


/**
 * @brief update 
 */
void dwe::update_data() {
    //======================================shift==================================//
    xt_.leftCols(num_state_)=xt_.rightCols(num_state_);
    xt_real_.leftCols(num_state_)=xt_real_.rightCols(num_state_);
    xt_real_f_.leftCols(num_state_)=xt_real_f_.rightCols(num_state_);
    xt_f_.leftCols(num_state_)=xt_f_.rightCols(num_state_);
    xt_vel_.leftCols(num_state_)=xt_vel_.rightCols(num_state_);
    xt_2origin_.leftCols(num_state_) = xt_2origin_.rightCols(num_state_);
    MHE_ugv2origin_.leftCols(num_state_)=MHE_ugv2origin_.rightCols(num_state_);

    // Calculate the dt_
    currentTime_= ros::Time::now();

    if (previousTime_ != ros::Time(0)) {
        deltaTime_.leftCols(num_state_) = deltaTime_.rightCols(num_state_);
        deltaTime_(0, num_state_) = (currentTime_ - previousTime_).toSec();
    }else{
        lost_initTime_ = ros::Time::now();
    }
    previousTime_ = currentTime_;

    if (LOOP_num_ >= num_state_ + 1) {
        dt_ = deltaTime_.sum() / (num_state_ + 1);
    } else {
        dt_ = deltaTime_.sum() / LOOP_num_;
    }
    // get data for every spin

    if (!hasStartEst_) {
        initializeEstimation();
    } else {
        flag_init_ok_ = true;
        ROS_WARN("----------- Start Estimate -----------");
    }

    A << 1, 0, 0, dt_, 0, 0,
                0, 1, 0, 0, dt_, 0,
                0, 0, 1, 0, 0, dt_,
                0, 0, 0, 1-MHE_kappa_(0)*dt_, 0, 0,
                0, 0, 0, 0, 1-MHE_kappa_(1)*dt_, 0,
                0, 0, 0, 0, 0, 1-MHE_kappa_(2)*dt_;

    B << 0.5*pow(dt_,2), 0, 0,
                    0, 0.5*pow(dt_,2), 0,
                    0, 0, 0.5*pow(dt_,2),
                    dt_, 0,  0,  
                    0,  dt_, 0,
                    0,  0, dt_; 

    //==================================IMU==================================//
    Eigen::MatrixXd cov_pre_xt = A * xt_f_.block(0, 0, 6, num_state_) + 
                                B * MHE_imu_.block(0, 1, 3, num_state_);
    Eigen::MatrixXd diff_xt_imu = xt_f_.block(0, num_state_, 6, 1) - 
                                cov_pre_xt.block(0, num_state_ -1, 6, 1);
    Eigen::MatrixXd cov_imu = (diff_xt_imu * diff_xt_imu.transpose());

    sensor.cov.imu_p = cov_imu.topRows(3).diagonal();
    sensor.cov.imu_v = cov_imu.bottomRows(3).diagonal();
    sensor.cov.imu_p = sensor.cov.imu_p.cwiseMin(100.0).cwiseMax(0.000001);
    sensor.cov.imu_v = sensor.cov.imu_v.cwiseMin(100.0).cwiseMax(0.000001);

    MHE_imu_.leftCols(num_state_) = MHE_imu_.rightCols(num_state_);
    MHE_imu_residual_.leftCols(num_state_) = MHE_imu_residual_.rightCols(num_state_);

    MHE_imu_att_.leftCols(num_state_) = MHE_imu_att_.rightCols(num_state_);
    rotation_matrix = quaternion2mat(MHE_imu_att_.col(num_state_));

    MHE_imu_(0,num_state_) = imu_msg.linear_acceleration.x * GRAVITATIONAL_ACC;
    MHE_imu_(1,num_state_) = imu_msg.linear_acceleration.y * GRAVITATIONAL_ACC;
    MHE_imu_(2,num_state_) = imu_msg.linear_acceleration.z * GRAVITATIONAL_ACC;

    MHE_imu_att_(0, num_state_) = imu_msg.orientation.w;
    MHE_imu_att_(1, num_state_) = imu_msg.orientation.x;
    MHE_imu_att_(2, num_state_) = imu_msg.orientation.y;
    MHE_imu_att_(3, num_state_) = imu_msg.orientation.z;

    MHE_imu_.col(num_state_) = rotation_matrix* MHE_imu_.col(num_state_);
    MHE_imu_(2, num_state_) += bias_imu_;

    cov_pre_xt_new = A * xt_.block(0, 0, 6, num_state_+1) + 
                    B * MHE_imu_.block(0, 0, 3, num_state_+1);

    //*****************imu: lost detection
    MHE_imu_residual_.col(num_state_) = (MHE_imu_.col(num_state_) - 
                                        MHE_imu_.col(num_state_-1)).cwiseAbs();
    sensor.fail.imu = data_loss_detection(MHE_imu_residual_.row(0),residual_threshold);

    //*****************imu: quality detection
    double k_imu = 10.0;  
    double x0_imu = 0.30;  
    sensor.quality.imu = Eigen::Vector3d::Ones() - 
                        sigmoid_eigen(MHE_imu_residual_.col(num_state_), k_imu, x0_imu);

    //==================================UWB==================================//
    MHE_uwb_.leftCols(num_state_) = MHE_uwb_.rightCols(num_state_);
    MHE_uwb_residual_.leftCols(num_state_) = MHE_uwb_residual_.rightCols(num_state_);
    
    MHE_uwb_bias_.leftCols(num_state_) = MHE_uwb_bias_.rightCols(num_state_);
    MHE_uwb_bias_.col(num_state_) = (quaternion2mat(MHE_imu_att_.col(num_state_)) * (bias_uwb_.row(0).transpose())).transpose();

    //*****************uwb: lost detection
    if (uwb_msg.nodes.size() == 1){
        MHE_uwb_(0,num_state_)=uwb_msg.nodes[0].dis + bias_d_;
        MHE_uwb_residual_.col(num_state_) = (MHE_uwb_.col(num_state_) - 
                                            MHE_uwb_.col(num_state_-1)).cwiseAbs();
        
        if (hasStartEst_) {
            if (MHE_uwb_residual_(0,num_state_) >= 0.5) {
                MHE_uwb_(0, num_state_) = MHE_uwb_(0, num_state_ - 1);
            }
        }
        sensor.fail.uwb = data_loss_detection(MHE_uwb_residual_,residual_threshold);

    }
    else{
        // uwb lost
        ROS_WARN("======MHE_uwb lOST!========");
        MHE_uwb_(0,num_state_)=MHE_uwb_(0,num_state_-1);
        MHE_uwb_residual_.col(num_state_) = (MHE_uwb_.col(num_state_) - 
                                            MHE_uwb_.col(num_state_-1)).cwiseAbs();
        sensor.fail.uwb = 0;
    }

    //*****************uwb: quality detection
    if (sensor.fail.uwb == 0) { 
        ROS_WARN("CAM failed!!!!!");
        sensor.quality.uwb = 0.001;
    }else{
        double k_uwb = 10.0; 
        double x0_uwb = 0.30; 
        sensor.quality.uwb = 1 - sigmoid(MHE_uwb_residual_(0,num_state_), k_uwb, x0_uwb);
    }

    Eigen::MatrixXd xt_norm = cov_pre_xt.topRows(3).colwise().norm();
    Eigen::MatrixXd diff_uwb = MHE_uwb_.block(0, num_state_, 1, 1) - xt_norm.block(0, num_state_-1, 1, 1);
    Eigen::MatrixXd cov_uwb = (diff_uwb * diff_uwb.transpose());
    sensor.cov.uwb = cov_uwb(0);
    sensor.cov.uwb = std::min(sensor.cov.uwb, 100.0);
    sensor.cov.uwb = std::max(sensor.cov.uwb, 0.000001);

    //==================================mocap posuav==================================//
    VICON_PosUAV_.leftCols(num_state_)=VICON_PosUAV_.rightCols(num_state_);
    VICON_PosUWB_.leftCols(num_state_)=VICON_PosUWB_.rightCols(num_state_);
    VICON_PosUAV_(0,num_state_)=VICON_posUAV_msg.pose.position.x + bias_marker_(0);
    VICON_PosUAV_(1,num_state_)=VICON_posUAV_msg.pose.position.y + bias_marker_(1);
    VICON_PosUAV_(2,num_state_)=VICON_posUAV_msg.pose.position.z + bias_marker_(2);

    VICON_PosUWB_(0,num_state_)=uwb_pos_msg.pose.position.x + bias_marker_ugv_(0);
    VICON_PosUWB_(1,num_state_)=uwb_pos_msg.pose.position.y + bias_marker_ugv_(1);
    VICON_PosUWB_(2,num_state_)=uwb_pos_msg.pose.position.z + bias_marker_ugv_(2);

    // mocap_uwb_ : relative dis
    VICON_uwb_.leftCols(num_state_)=VICON_uwb_.rightCols(num_state_);
    VICON_uwb_(0,num_state_) = sqrt(pow(VICON_PosUAV_(0,num_state_)+MHE_uwb_bias_(0,num_state_)-VICON_PosUWB_(0,num_state_),2)+
    pow(VICON_PosUAV_(1,num_state_)+MHE_uwb_bias_(1,num_state_)-VICON_PosUWB_(1,num_state_),2)+
    pow(VICON_PosUAV_(2,num_state_)+MHE_uwb_bias_(2,num_state_)-VICON_PosUWB_(2,num_state_),2));

    // mocap veluav
    VICON_VelUAV_.leftCols(num_state_)=VICON_VelUAV_.rightCols(num_state_);
    VICON_VelUAV_(0,num_state_)=VICON_velUAV_msg.twist.linear.x;
    VICON_VelUAV_(1,num_state_)=VICON_velUAV_msg.twist.linear.y;
    VICON_VelUAV_(2,num_state_)=VICON_velUAV_msg.twist.linear.z;

    VICON_VelUWB_.leftCols(num_state_)=VICON_VelUWB_.rightCols(num_state_);
    if (dt_== 0) {
        // First message received, initialize previousTime
        VICON_VelUWB_(2,num_state_)=0;
    } else{
        VICON_VelUWB_(0,num_state_)=(VICON_PosUWB_(0,num_state_)-VICON_PosUWB_(0,num_state_-1))/dt_;
        VICON_VelUWB_(1,num_state_)=(VICON_PosUWB_(1,num_state_)-VICON_PosUWB_(1,num_state_-1))/dt_;
        VICON_VelUWB_(2,num_state_)=(VICON_PosUWB_(2,num_state_)-VICON_PosUWB_(2,num_state_-1))/dt_;
    }

    ROS_INFO("EST MHE_cam1_: cam01_x:%f cam01_y:%f cam01_z:%f",
        MHE_cam_(0,num_state_),
        MHE_cam_(1,num_state_),
        MHE_cam_(2,num_state_));

    MHE_ugv2origin_.rightCols(1) = VICON_PosUWB_.rightCols(1) - VICON_uwb_ini.col(0);
    VICON_PosUAV2UWB_.block(0,0,3,1)= VICON_PosUAV_.rightCols(1) - VICON_PosUWB_.rightCols(1);
    VICON_PosUAV2UWB_.block(3,0,3,1)= VICON_VelUAV_.rightCols(1) - VICON_VelUWB_.rightCols(1);
    VICON_PosUAV2_fixedorigin_ = VICON_PosUAV_.rightCols(1) - VICON_uwb_ini.col(0);

    //==================================FLOW==================================//
    MHE_flow_.leftCols(num_state_)=MHE_flow_.rightCols(num_state_);
    MHE_flow_residual_.leftCols(num_state_)=MHE_flow_residual_.rightCols(num_state_);
    MHE_reletive_vel_.leftCols(num_state_)=MHE_reletive_vel_.rightCols(num_state_);
    
    MHE_height_.leftCols(num_state_)=MHE_height_.rightCols(num_state_);
    MHE_reletive_height_.leftCols(num_state_)=MHE_reletive_height_.rightCols(num_state_);
    MHE_hgt_residual_.leftCols(num_state_)=MHE_hgt_residual_.rightCols(num_state_);

    flow_quality_ = flow_msg.quality;
    MHE_height_(0,num_state_) = - flow_msg.distance + 0.12 -0.005;

    //*****************hgt: lost detection
    MHE_hgt_residual_.col(num_state_) = (MHE_height_.col(num_state_) - 
                                        MHE_height_.col(num_state_-1)).cwiseAbs();
    
    if (MHE_hgt_residual_(0,num_state_) >= 0.50) {
        MHE_height_(0,num_state_) = MHE_height_(0,num_state_-1);
    } else if (MHE_hgt_residual_(0,num_state_) >= 0.35) {
        MHE_height_(0,num_state_) = (MHE_height_(0,num_state_-1)+MHE_height_(0,num_state_))/2;
    }
    // check
    if (MHE_height_(0, num_state_) < 0.4) {
        paramFLOW_.block<3, 1>(0, num_state_) *= 0.3;
    }
    MHE_reletive_height_(0,num_state_) = MHE_height_(0,num_state_) - VICON_uwb_ini(2,0);

    // update 
    MHE_flow_(0,num_state_)= -flow_msg.integrated_x;
    MHE_flow_(1,num_state_)= -flow_msg.integrated_y;
    MHE_flow_.col(num_state_) = rotation_matrix * MHE_flow_.col(num_state_);

    if (dt_== 0) {
        MHE_flow_(2,num_state_)=0;
    } else{
        MHE_flow_(2,num_state_)=(MHE_height_(0,num_state_)-MHE_height_(0,num_state_-1))/dt_;
    }
    MHE_reletive_vel_.col(num_state_) = MHE_flow_.col(num_state_) - VICON_VelUWB_.col(num_state_);

    //*****************flow: lost detection
    MHE_flow_residual_.col(num_state_) = (MHE_flow_.col(num_state_) - 
                                        MHE_flow_.col(num_state_-1)).cwiseAbs();
    sensor.fail.flow = data_loss_detection(MHE_flow_residual_.row(0),residual_threshold);
    sensor.fail.hgt = sensor.fail.flow;

    //*****************flow: quality detection
    if (sensor.fail.flow == 0) { 
        ROS_WARN("FLOW failed!!!!!");
        sensor.quality.flow = Eigen::Vector3d(0.001, 0.001, 0.001);
        sensor.quality.hgt = 0.001;
    }else{
        double flow_quality_norm =  (flow_quality_ + 5) / 255.0;
        double k_flow = 10.0; 
        double x0_flow = 0.95; 
        sensor.quality.base = sigmoid(flow_quality_norm, k_flow, x0_flow);

        double x0_flow_data = 0.30; 
        sensor.quality.flow = Eigen::Vector3d::Ones() - 
            sigmoid_eigen(MHE_flow_residual_.col(num_state_), k_flow, x0_flow_data);

        //*****************hgt: quality detection
        double k_hgt = 10.0;
        double x0_hgt = 0.30; 
        sensor.quality.hgt = 1 - sigmoid(MHE_hgt_residual_(0,num_state_), k_hgt, x0_hgt);
    }

    Eigen::MatrixXd diff_hgt = MHE_reletive_height_.block(0, num_state_, 1, 1) - 
                                cov_pre_xt.block(2, num_state_-1, 1, 1);
    Eigen::MatrixXd cov_hgt = (diff_hgt * diff_hgt.transpose());
    sensor.cov.hgt = cov_hgt(0);
    sensor.cov.hgt = std::min(sensor.cov.hgt, 100.0);
    sensor.cov.hgt = std::max(sensor.cov.hgt, 0.000001);

    Eigen::MatrixXd diff_flow = MHE_reletive_vel_.block(0, num_state_, 3, 1) - cov_pre_xt.block(3, num_state_-1, 3, 1);
    Eigen::MatrixXd cov_flow = (diff_flow * diff_flow.transpose());
    sensor.cov.flow = cov_flow.diagonal();
    sensor.cov.flow = sensor.cov.flow.cwiseMin(100.0).cwiseMax(0.000001);
    //==================================CAM==================================//
    MHE_cam_.leftCols(num_state_)=MHE_cam_.rightCols(num_state_);
    MHE_cam_residual_.leftCols(num_state_)=MHE_cam_residual_.rightCols(num_state_);
    int cam_fail_flag = 0;

    for (int i=0;i<camnum_;i++){
        if (std::isnan(cam_cbs_[i].transform.translation.x)) {
            cam_cbs_[i].transform.translation.x = 0;
            cam_cbs_[i].transform.translation.y = 0;
            cam_cbs_[i].transform.translation.z = 0;
        }

        MHE_cam_(3 * i, num_state_)     = cam_cbs_[i].transform.translation.x + bias_cam_(0,i);
        MHE_cam_(3 * i + 1, num_state_) = cam_cbs_[i].transform.translation.y + bias_cam_(1,i);
        MHE_cam_(3 * i + 2, num_state_) = cam_cbs_[i].transform.translation.z + bias_cam_(2,i); 

        //*****************cam: lost detection
        MHE_cam_residual_.col(num_state_) = (MHE_cam_.block(3 * i,num_state_,3,1) - 
                                MHE_cam_.block(3 * i,num_state_-1,3,1)).cwiseAbs();
        sensor.fail.cam = data_loss_detection(MHE_cam_residual_.row(0),residual_threshold);

        Eigen::Vector3d diff = (MHE_cam_.block<3, 1>(3 * i, num_state_) - MHE_cam_.block<3, 1>(3 * i, num_state_ - 1)).cwiseAbs();
        Eigen::Vector3d diffxt = (MHE_cam_.block<3, 1>(3 * i, num_state_) - xt_f_.block<3, 1>(0, num_state_)).cwiseAbs();
        double diffx0 = abs(MHE_cam_(3 * i+2, num_state_-1) - MHE_cam_(3 * i+2, 0));

        if (hasStartEst_) {
            bool diff_zero = (diff.array() == 0.000000).any();
            bool diff_large = (diff.array() >= 0.30).any();
            bool diffxt_large = (diffxt.array() >= 0.40).any();
        
            if (diff_zero) {
                if(diffx0 == 0.000000 || diff_large || diffxt_large){
                    ROS_WARN("CAM %d failed, diffxt[0] = %.4f, diffxt[1] = %.4f, diffxt[2] = %.4f",i+1, diffxt[0],diffxt[1],diffxt[2] );
                    sensor.fail.cam = 0;
                    continue;
                }
            }

            if(!diff_zero && (diffxt.array() <= 0.800).all() && diffx0 == 0.00000){
                continue;
            }
            if (diff_large || (diffxt.array() >=3.00).any()) {
                MHE_cam_.block<3, 1>(3 * i, num_state_) = MHE_cam_.block<3, 1>(3 * i, num_state_ - 1);
                sensor.fail.cam = 0;
                continue;
            }
        }
    }

    // //===============used for long time lost - L2
    // if ((currentTime_ - lost_initTime_).toSec() >=30.0 && (currentTime_ - lost_initTime_).toSec() <=40.0 || 
    // (currentTime_ - lost_initTime_).toSec() >=50.0 && (currentTime_ - lost_initTime_).toSec() <=60.0) {  // 确保数据不为空
    //     sensor.fail.cam = 0;
    //     MHE_cam_.block<3, 1>(0, num_state_) = MHE_cam_.block<3, 1>(0, num_state_ - 1);
    //     ROS_WARN("==================CAM SET TO FAIL==============");
    // }
    // //===============used for long time lost - L2

    // //===============used for long time lost v2 - L1
    // if ((currentTime_ - lost_initTime_).toSec() >=13.0 && (currentTime_ - lost_initTime_).toSec() <=28.0) {  // 确保数据不为空
    //     sensor.fail.cam = 0;
    //     MHE_cam_.block<3, 1>(0, num_state_) = MHE_cam_.block<3, 1>(0, num_state_ - 1);
    //     ROS_WARN("==================CAM SET TO FAIL==============");
    // }
    // //===============used for long time lost - L1

    //*****************cam: quality detection
    if (sensor.fail.cam == 0) { 
        ROS_WARN("CAM failed!!!!!");
        sensor.quality.cam = Eigen::Vector3d(0.001, 0.001, 0.001);
    }else{
        double k_cam = 10.0; 
        double x0_cam = 0.10; 
        sensor.quality.cam = Eigen::Vector3d::Ones() - 
            sigmoid_eigen(MHE_cam_residual_.col(num_state_), k_cam, x0_cam);
    }

    Eigen::MatrixXd diff_cam = MHE_cam_.block(0, num_state_, 3, 1) - 
                                    cov_pre_xt.block(0, num_state_-1, 3, 1);
    Eigen::MatrixXd cov_cam = (diff_cam * diff_cam.transpose());
    sensor.cov.cam = cov_cam.diagonal();
    sensor.cov.cam = sensor.cov.cam.cwiseMin(100.0).cwiseMax(0.000001);

    //==================================set param==================================//
    paramCAM_.leftCols(num_state_)=paramCAM_.rightCols(num_state_);
    paramFLOW_.leftCols(num_state_)=paramFLOW_.rightCols(num_state_);
    paramUWB_.leftCols(num_state_)=paramUWB_.rightCols(num_state_);
    paramHGT_.leftCols(num_state_)=paramHGT_.rightCols(num_state_);
    paramIMU_.leftCols(num_state_-1)=paramIMU_.rightCols(num_state_-1);
    // ================================= cov =======================================//

    sum_cov.cov.uwb = sensor.cov.uwb * sensor.fail.uwb;
    sum_cov.cov.hgt = sensor.cov.hgt * sensor.fail.hgt;
    for (int i = 0; i < 3; ++i) {
        sum_cov.cov.cam[i] = sensor.cov.cam[i] * sensor.fail.cam;
        sum_cov.cov.flow[i] = sensor.cov.flow[i] * sensor.fail.flow;
        sum_cov.cov.imu_p[i] = sensor.cov.imu_p[i] * sensor.fail.imu;
        sum_cov.cov.imu_v[i] = sensor.cov.imu_v[i] * sensor.fail.imu;
    }
    sum_cov.cov.sum = sum_cov.cov.uwb + sum_cov.cov.hgt + 
                      sum_cov.cov.imu_p.sum()  + sum_cov.cov.imu_v.sum() +
                      sum_cov.cov.cam.sum() + 
                      sum_cov.cov.flow.sum() + 0.001;

    sensor.cov_per.hgt =
    (1-sum_cov.cov.hgt/ sum_cov.cov.sum);

    sensor.cov_per.uwb =
    (1-sum_cov.cov.uwb/sum_cov.cov.sum);

    for(int i = 0;i<3;++i){
        sensor.cov_per.cam[i] = (1-sum_cov.cov.cam[i]/sum_cov.cov.sum);
        sensor.cov_per.flow[i] = (1-sum_cov.cov.flow[i]/sum_cov.cov.sum);
        sensor.cov_per.imu_p[i] = (1-sum_cov.cov.imu_p[i]/sum_cov.cov.sum);
        sensor.cov_per.imu_v[i] = (1-sum_cov.cov.imu_v[i]/sum_cov.cov.sum);
    }
    // percent
    sensor.weight.uwb = sensor.quality.uwb *  sensor.fail.uwb * sensor.cov_per.uwb;
    sensor.weight.hgt = sensor.quality.hgt *  sensor.fail.hgt * sensor.cov_per.hgt;
    for (int i = 0; i < 3; ++i) {
        sensor.weight.cam[i]   = sensor.quality.cam[i]  *  sensor.fail.cam  * sensor.cov_per.cam[i];
        sensor.weight.flow[i]  = sensor.quality.flow[i] *  sensor.fail.flow * sensor.cov_per.flow[i];
        sensor.weight.imu_p[i] = sensor.quality.imu[i]  *  sensor.fail.imu  * sensor.cov_per.imu_p[i];
        sensor.weight.imu_v[i] = sensor.quality.imu[i]  *  sensor.fail.imu  * sensor.cov_per.imu_v[i];
    } 

    sensor.weight.sum = sensor.weight.uwb + sensor.weight.hgt +
                         sensor.weight.imu_p.sum() + sensor.weight.imu_v.sum() + 
                         sensor.weight.cam.sum() +
                         sensor.weight.flow.sum() + 0.001;

   // ============== adparam
    auto normalizeAndLimit = [&](double weight) {
        return std::min((weight / sensor.weight.sum) * sensor.factor.sum, 100.0);
    };

    paramUWB_(0, num_state_) = normalizeAndLimit(sensor.weight.uwb);
    paramHGT_(0, num_state_) = normalizeAndLimit(sensor.weight.hgt);
    for (int i = 0; i < 3; ++i) {
        paramCAM_(i, num_state_) = normalizeAndLimit(sensor.weight.cam[i]);
        paramFLOW_(i, num_state_) = normalizeAndLimit(sensor.weight.flow[i]);
        paramIMU_(i, num_state_ - 1) = normalizeAndLimit(sensor.weight.imu_p[i]);
        paramIMU_(i+3, num_state_ - 1) = normalizeAndLimit(sensor.weight.imu_v[i]);
    }

    // //  display
    // ROS_WARN("paramUWB_: %f", paramUWB_(0, num_state_));
    // ROS_WARN("paramHGT_: %f", paramHGT_(0, num_state_));
    // ROS_WARN("paramFLOW_: %f, %f, %f",paramFLOW_(0, num_state_),paramFLOW_(1, num_state_),paramFLOW_(2, num_state_));
    // ROS_WARN("paramCAM_: %f, %f, %f",paramCAM_(0, num_state_),paramCAM_(1, num_state_),paramCAM_(2, num_state_));
    // ROS_WARN("paramIMU_: %f, %f, %f,%f, %f, %f",paramIMU_(0, num_state_-1),paramIMU_(1, num_state_-1),paramIMU_(2, num_state_-1),
    // paramIMU_(3, num_state_-1),paramIMU_(4, num_state_-1),paramIMU_(5, num_state_-1));
}


int data_loss_detection(const Eigen::MatrixXd& MHE_data_, double loss_epsilon) {
    double sum_of_squares = 0;
    int MHE_data_num = MHE_data_.cols();
    
    for (int i = 0; i < MHE_data_num; i++) {
        sum_of_squares += MHE_data_(0, i) * MHE_data_(0, i);
    }
    
    if (double (sum_of_squares / MHE_data_num) < loss_epsilon) {
        return 0;
    } else {
        return 1;
    }
}


double sigmoid(double x, double k, double x0) {
    return 1.0 / (1.0 + exp(-k * (x - x0)));
}

Eigen::MatrixXd sigmoid_eigen(const Eigen::MatrixXd& x, 
                       double k, 
                       double x0) {
    return (1.0 / (1.0 + (-k * (x.array() - x0)).exp())).matrix();
}


/**
 * @brief in sliding window
 */
void dwe::solveonce()
{
    ROS_INFO("dt: %.4lf seconds", dt_); 
    x1_ = cov_pre_xt_new;

    MatrixXd imu_sol;
    MatrixXd flow_sol;
    MatrixXd uwb_sol;
    MatrixXd hgt_sol;
    MatrixXd cam_sol;

    MatrixXd paramXx0_  = Eigen::MatrixXd::Zero(6, num_state_);

    if (vicon_flag_ == 1)
    {
        imu_sol = MHE_imu_.leftCols(k_*num_state_);
        uwb_sol = VICON_uwb_;
        MatrixXd relative_pos;
        relative_pos = VICON_PosUAV_ - VICON_PosUWB_; 

        flow_sol= VICON_VelUAV_ - VICON_VelUWB_; 
        cam_sol = relative_pos;
        hgt_sol = relative_pos.row(2);

        MHE_flow_= VICON_VelUAV_ - VICON_VelUWB_;
        MHE_uwb_ = VICON_uwb_;
        MHE_height_ = relative_pos.row(2);
        MHE_cam_= relative_pos; 
    }
    else{

        imu_sol = MHE_imu_.leftCols(k_*num_state_);
        flow_sol = MHE_reletive_vel_; 
        uwb_sol = MHE_uwb_;
        hgt_sol = MHE_reletive_height_;
        cam_sol = MHE_cam_;
    }
    
    A << 1, 0, 0, dt_, 0, 0,
                0, 1, 0, 0, dt_, 0,
                0, 0, 1, 0, 0, dt_,
                0, 0, 0, 1-MHE_kappa_(0)*dt_, 0, 0,
                0, 0, 0, 0, 1-MHE_kappa_(1)*dt_, 0,
                0, 0, 0, 0, 0, 1-MHE_kappa_(2)*dt_;

    B << 0.5*pow(dt_,2), 0, 0,
                    0, 0.5*pow(dt_,2), 0,
                    0, 0, 0.5*pow(dt_,2),
                    dt_, 0,  0,  
                    0,  dt_, 0,
                    0,  0, dt_; 

    AA = A;
    for (int i = 1; i < k_; i++) {
        AA = AA*A;
    }  

    for (int i=1;i<=num_state_+1;i++){
        // x0- ---- Ex Eu W
        Ex.block((i-1)*6,(i-1)*6,6,6)=Eigen::MatrixXd::Identity(6,6);
        Eu.block((i-1)*6,(i-1)*6,6,6) = Eigen::MatrixXd::Identity(6,6);
        W.block((i-1)*6,(i-1)*6,6,6) = paramX0_.col(i-1).asDiagonal();

        if(i!=num_state_+1){
            // imu ---- Ex Eu W
            Ex.block((i-1)*6+6*(num_state_+1),(i-1)*6,6,6) = -AA;
            Ex.block((i-1)*6+6*(num_state_+1),(i)*6,6,6) = Eigen::MatrixXd::Identity(6,6);
            for(int j=1;j<=k_;j++){
                Eigen::Matrix<double, 6, 6> Akj;
                Akj.setIdentity(6,6);
                int kj=k_-j;
                for (int q=0;q<kj;q++){
                    Akj=Akj*A;
                }
                Eu.block(6*(num_state_+1)+(i-1)*6,6*(num_state_+1)+(i-1)*3*k_+(j-1)*3,6,3) = Akj*B;
            }
            W.block(6*(num_state_+1)+(i-1)*6,6*(num_state_+1)+(i-1)*6,6,6) = paramIMU_.col(i-1).asDiagonal();

            Eigen::MatrixXd tt(1,order_+1);
            for(int j=1;j<=(order_+1);j++){
                tt(0,j-1) = pow(dt_*i*k_,j-1);
            }
            for(int j=1;j<=6;j++){
                t_matrix.block((i)*6+j-1,(j-1)*(order_+1),1,order_+1) = tt;
            }
        } 

        // measurements
        Eigen::MatrixXd x0_loc=x1_.topRows(3).col(i-1);
        double rho=x0_loc.norm();
        Eigen::MatrixXd C(8,6);
        C.setZero(8,6);
        C.block(0,0,1,3) = x0_loc.transpose()/rho;
        C.block(1,3,3,3) = Eigen::MatrixXd::Identity(3,3);
        C.block(4,0,3,3) = Eigen::MatrixXd::Identity(3,3);

        Eigen::RowVector3d Zh_(0,0,1);
        C.block(7,0,1,3) = Zh_;
        Ex.block((num_state_+1)*6+6*(num_state_+1)+(i-1)*8,(i-1)*6,8,6) = C;
        Eu.block(((num_state_+1))*12+8*(i-1),(num_state_+1)*3*k_+6*(num_state_+1)+8*(i-1),8,8) = Eigen::MatrixXd::Identity(8,8);

        Eigen::VectorXd term0(6); 
        term0.setZero();
        term0.segment(0, 3) = MHE_uwb_bias_.col(i-1).transpose();
        Eigen::VectorXd term2 = uwb_sol(0,i*k_-1)* Eigen::VectorXd::Ones(1) - C.block(0,0,1,6) * term0; 
        ur(3*k_*(num_state_+1)+6*(num_state_+1)+(i-1)*8,0) = term2(0);
        ur.block(3*k_*(num_state_+1)+6*(num_state_+1)+(i-1)*8+1,0,3,1) = flow_sol.col(i*k_-1);
        ur.block(3*k_*(num_state_+1)+6*(num_state_+1)+(i-1)*8+4,0,3,1) = cam_sol.col(i*k_-1);
        ur(3*k_*(num_state_+1)+6*(num_state_+1)+(i-1)*8+7,0) = hgt_sol(0,i*k_-1);

        W(6*(num_state_+1)+(num_state_+1)*6+8*(i-1),6*(num_state_+1)+(num_state_+1)*6+8*(i-1)) = paramUWB_(0,i-1); 
        W.block(6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+1,6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+1,3,3) = paramFLOW_.col(i-1).asDiagonal();
        W.block(6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+4,6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+4,3,3) = paramCAM_.col(i-1).asDiagonal();
        W(6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+7,6*(num_state_+1)+(num_state_+1)*6+8*(i-1)+7) = paramHGT_(0,i-1);
    }

    xtt=x1_;
    for(int j=1;j<=xtt.cols();j=j+1){
        ur.block(0+xtt.rows()*(j-1),0,xtt.rows(),1) = xtt.col(j-1);
    }

    for(int j=1;j<=imu_sol.cols();j=j+1){
        ur.block(xtt.cols()*xtt.rows()+imu_sol.rows()*(j-1),0,imu_sol.rows(),1) = imu_sol.col(j-1);
    }

    Ex_new = Ex * t_matrix; 
    ar = ((Ex_new.transpose()*W*Ex_new).inverse())*(Ex_new.transpose())*W*Eu*ur;

    xxt_new = t_matrix * ar;

    for(int i=1;i<=num_state_+1;i++){
        xxe.block(0,i-1,6,1) = xxt_new.block(6*(i-1),0,6,1);
    }

    xt_.rightCols(1)=xxe.rightCols(1);

    xt_real_(0,num_state_) = xt_(0,num_state_) + VICON_uwb_ini(0,0) + bias_pos_(0);
    xt_real_(1,num_state_) = xt_(1,num_state_) + VICON_uwb_ini(1,0) + bias_pos_(1);
    xt_real_(2,num_state_) = xt_(2,num_state_) + VICON_uwb_ini(2,0) + bias_pos_(2);

    xt_real_(3,num_state_) = xt_(3,num_state_) + VICON_VelUWB_(0,num_state_);
    xt_real_(4,num_state_) = xt_(4,num_state_) + VICON_VelUWB_(1,num_state_);
    xt_real_(5,num_state_) = xt_(5,num_state_) + VICON_VelUWB_(2,num_state_);

    Eigen::MatrixXd xt_real_meanFilter = meanFilter(xt_real_.row(5).rightCols(3));
    xt_real_.row(5).rightCols(1) = xt_real_meanFilter.rightCols(1);
    
    xt_f_(0,num_state_) = xt_(0,num_state_) + bias_pos_(0);
    xt_f_(1,num_state_) = xt_(1,num_state_) + bias_pos_(1);
    xt_f_(2,num_state_) = xt_(2,num_state_) + bias_pos_(2);
    
    Eigen::MatrixXd xt_pos_meanFilter = meanFilter(xt_f_.topRows(2).rightCols(3));
    xt_f_.topRows(2).rightCols(1) = xt_pos_meanFilter.rightCols(1);
    xt_2origin_.rightCols(1) = xt_f_.topRows(3).rightCols(1) + MHE_ugv2origin_.rightCols(1);

    int mean_num = 3;
    for (int i = 1; i <= 3; i++) {
        xt_vel_(i-1,num_state_)=(xt_f_(i-1,num_state_)-xt_f_(i-1,num_state_-1))/dt_;
        if(abs(xt_vel_(i-1,num_state_) - xt_vel_(i-1,num_state_-1))>=5.0){
            xt_vel_(i-1,num_state_) = xt_vel_(i-1,num_state_-1);
        }

    }
    Eigen::MatrixXd xt_posf_meanFilter = meanFilter(xt_vel_.rightCols(3));
    xt_f_.bottomRows(3).rightCols(1) = xt_posf_meanFilter.rightCols(1);

    return;

}


//========================callback functions=======================================//
/**
 * @brief callback from uwb3
 */
void dwe::uwb3_cb(const nlink_parser::LinktrackNodeframe3::ConstPtr &msg){
    // flag_init_imu_ = true;
    uwb_msg = *msg;
}

/**
 * @brief dynamic uav
 */
void dwe::pos_uav_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){  // dynamic uav

    VICON_posUAV_msg = *msg;
}

void dwe::vel_uav_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){       // dynamic uav

    VICON_velUAV_msg = *msg;
}

void dwe::vel_uwb_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){       // dynamic uav

    VICON_velUWB_msg = *msg;
}


/**
 * @brief callback from vicon-UWB01
 */
void dwe::pos_uwba0_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){  // static uav

    uwb_pos_msg = *msg;
}

/**
 * @brief callback from cam
 */
void dwe::camA_cb(const geometry_msgs::TransformStamped::ConstPtr &msg){

    cam_cbs_[0] = *msg;
    cam_data_received_ = true;
}

void dwe::flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &msg){
    flag_init_imu_ = true;
    flow_msg = *msg;
}

void dwe::state_cam_cb(const std_msgs::Float32MultiArray::ConstPtr &msg){
    state_cam_msg = *msg;
}