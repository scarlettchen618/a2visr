#include "a2VISR.h"

using namespace Eigen;
using namespace std;    

ros::Publisher estimate_pose_pub;
ros::Publisher estimate_vel_pub;
ros::Publisher estimate_pose_f_pub;
ros::Publisher estimate_vel_f_pub;
ros::Publisher estimate_viconpose_pub;
ros::Publisher estimate_viconvel_pub;
ros::Publisher estimate_orientation_pub;

ros::Publisher estimate_relpose_pub;
ros::Publisher estimate_relvel_pub;

ros::Publisher param_uwb_pub;
ros::Publisher param_cam_pub;
ros::Publisher param_flow_pub;
ros::Publisher param_imu_pub;
ros::Publisher param_cov_pub;
ros::Publisher kappa_pub;
ros::Publisher param_quality_pub;
ros::Publisher param_fail_pub;

ros::Publisher uesd_imu_pub;
ros::Publisher uesd_hgt_pub;
ros::Publisher uesd_uwb_pub;
ros::Publisher uesd_uwbvicon_pub;
ros::Publisher uesd_dt_pub;
ros::Publisher uesd_flow_pub;
ros::Publisher uesd_rel_flow_pub;
ros::Publisher camera_A_pub;
ros::Publisher estimate_est_relpose_pub;
ros::Publisher estimate_est_relvel_pub;
ros::Publisher estimate_relpose2origin_pub;
ros::Publisher estimate_viconrelpose2origin_pub;

// ================================ Function Declaration and Definition ========================= //
void PubPose(const MatrixXd& data, int num, ros::Publisher& publisher, int start_index) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = data(start_index, num);
    pose_msg.pose.position.y = data(start_index + 1, num);
    pose_msg.pose.position.z = data(start_index + 2, num);
    publisher.publish(pose_msg);
}

void PubHgt(const MatrixXd& data, int num, ros::Publisher& publisher) {
    geometry_msgs::PoseStamped hgt_msg;
    hgt_msg.header.stamp = ros::Time::now();
    hgt_msg.pose.position.x = data(0, num);
    publisher.publish(hgt_msg);
}

void PubPoseandOri(MatrixXd data,MatrixXd data_ori, int num, ros::Publisher& publisher, int start_index) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = data(start_index, num);
    pose_msg.pose.position.y = data(start_index + 1, num);
    pose_msg.pose.position.z = data(start_index + 2, num);

    pose_msg.pose.orientation.x = data_ori(start_index + 1, num);
    pose_msg.pose.orientation.y = data_ori(start_index + 2, num);
    pose_msg.pose.orientation.z = data_ori(start_index + 3, num);
    pose_msg.pose.orientation.w = data_ori(start_index, num);

    publisher.publish(pose_msg);
}

void PubVelocity(const MatrixXd& data, int num, ros::Publisher& publisher, int start_index) {
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.twist.linear.x = data(start_index, num);
    vel_msg.twist.linear.y = data(start_index + 1, num);
    vel_msg.twist.linear.z = data(start_index + 2, num);
    publisher.publish(vel_msg);
}

void Pubuwb(const MatrixXd& data, dwe *filter,ros::Publisher& publisher) {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(filter->uwbnum_);

    for(int i = 0; i < filter->uwbnum_; i++) {
        msg.data[i] = data(i, filter->num_state_);
    }

    publisher.publish(msg);
}

void Pubdt(float _used_dt,float _used_dt_solveonce ,dwe *filter){
    std_msgs::Float32MultiArray dt_msg;
    dt_msg.data.resize(2); 

    dt_msg.data[0] = _used_dt;
    dt_msg.data[1] = _used_dt_solveonce;

    uesd_dt_pub.publish(dt_msg); 
}

void Pubparamuwb(MatrixXd _uwb_param, dwe *filter) {
    std_msgs::Float32MultiArray param_uwb_msg;
    param_uwb_msg.data.resize(1); 

    param_uwb_msg.data[0] = _uwb_param(0, filter->num_state_);
    param_uwb_pub.publish(param_uwb_msg);
}

void Pubparamflow(MatrixXd _flow_param,MatrixXd _hgt_param, dwe *filter) {
    std_msgs::Float32MultiArray param_flow_msg;
    param_flow_msg.data.resize(4); 

    param_flow_msg.data[0] = _flow_param(0, filter->num_state_);
    param_flow_msg.data[1] = _flow_param(1, filter->num_state_);
    param_flow_msg.data[2] = _flow_param(2, filter->num_state_);
    param_flow_msg.data[3] = _hgt_param(0, filter->num_state_);

    param_flow_pub.publish(param_flow_msg);
}

void Pubparamcam(MatrixXd _cam_param, dwe *filter) {
    std_msgs::Float32MultiArray param_cam_msg;
    param_cam_msg.data.resize(3*(filter->camnum_)); 

    for(int i=0; i<filter->camnum_;i++){
        param_cam_msg.data[3*i]   = _cam_param(3*i, filter->num_state_);
        param_cam_msg.data[3*i+1] = _cam_param(3*i+1, filter->num_state_);
        param_cam_msg.data[3*i+2] = _cam_param(3*i+2, filter->num_state_);
    }
    param_cam_pub.publish(param_cam_msg);
}

void Pubparamimu(MatrixXd _imu_param, dwe *filter) {
    std_msgs::Float32MultiArray param_imu_msg;
    param_imu_msg.data.resize(6);

    for(int i=0; i<6;i++){
        param_imu_msg.data[i] = _imu_param(i, filter->num_state_-1);
    }
    param_imu_pub.publish(param_imu_msg);
}

void Pubkappa(MatrixXd _kappa,double _fac) {
    std_msgs::Float32MultiArray kappa_msg;
    kappa_msg.data.resize(4); 

    kappa_msg.data[0] = _kappa(0);
    kappa_msg.data[1] = _kappa(1);
    kappa_msg.data[2] = _kappa(2);
    kappa_msg.data[3] = _fac;
    kappa_pub.publish(kappa_msg);
}

void Pubcov(Vector3d _imu_p_cov,Vector3d _imu_v_cov, Vector3d _flow_cov,Vector3d _cam_cov, double _hgt_cov, double _uwb_cov,ros::Publisher& publisher) {
    std_msgs::Float32MultiArray cov_msg;
    cov_msg.data.resize(14);

    for(int i=0; i<3;i++){
        cov_msg.data[i]     = _imu_p_cov[i];  
        cov_msg.data[i+3]   = _imu_v_cov[i]; 
        cov_msg.data[i+6]   = _flow_cov[i]; 
        cov_msg.data[i+9]   = _cam_cov[i];  
    }
    cov_msg.data[12]  = _hgt_cov;
    cov_msg.data[13] = _uwb_cov;

    publisher.publish(cov_msg);
}

void Pubfail(int _imu_fail, int _flow_fail,int _cam_fail, int _hgt_fail, int _uwb_fail,ros::Publisher& publisher) {
    std_msgs::Float32MultiArray fail_msg;
    fail_msg.data.resize(5);

    fail_msg.data[0] = _imu_fail; 
    fail_msg.data[1] = _flow_fail;
    fail_msg.data[2] = _cam_fail; 
    fail_msg.data[3] = _hgt_fail;  
    fail_msg.data[4]  = _uwb_fail;

    publisher.publish(fail_msg);
}

void Pubquality(MatrixXd _imu_q, Vector3d _flow_q,Vector3d _cam_q, double _hgt_q, double _uwb_q, double _base_q,ros::Publisher& publisher) {
    std_msgs::Float32MultiArray q_msg;
    q_msg.data.resize(12); 

    for(int i=0; i<3;i++){
        q_msg.data[i]     = _imu_q(i); 
        q_msg.data[i+3]   = _flow_q[i]; 
        q_msg.data[i+6]   = _cam_q[i];  
    }
    q_msg.data[9]  = _hgt_q;
    q_msg.data[10] = _uwb_q;
    q_msg.data[11] = _base_q;

    publisher.publish(q_msg);
}
// ================================ Main Function ========================= //
int main (int argc, char** argv) 
{
    ros::init(argc, argv, "a2VISR", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(25);
    dwe dwe_filter(nh);

    //==================== subscriber =====================//
    ros::Subscriber imu_new_sub = nh.subscribe<sensor_msgs::Imu>("/IMU_data",1,boost::bind(&dwe::imu_new_cb, &dwe_filter, _1));
    ros::Subscriber uwb3_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3>("/nlink_linktrack_nodeframe3",1,boost::bind(&dwe::uwb3_cb, &dwe_filter, _1));
    ros::Subscriber camA_sub = nh.subscribe<geometry_msgs::TransformStamped>("/camA/single_cam_process_ros/ir_mono/T_base_to_estimation", 1,boost::bind(&dwe::camA_cb, &dwe_filter, _1));
    ros::Subscriber flow_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad", 1,boost::bind(&dwe::flow_cb, &dwe_filter, _1));

    ros::Subscriber pos_uav_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav201/mocap/pos",1,boost::bind(&dwe::pos_uav_cb, &dwe_filter, _1));
    ros::Subscriber vel_uwb_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uwba0/mocap/vel",1,boost::bind(&dwe::vel_uwb_cb, &dwe_filter, _1));
    ros::Subscriber vel_uav_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav201/mocap/vel", 1,boost::bind(&dwe::vel_uav_cb, &dwe_filter, _1));
    ros::Subscriber pos_uwba0_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uwba0/mocap/pos",1,boost::bind(&dwe::pos_uwba0_cb, &dwe_filter, _1));

    ros::Subscriber state_cam_sub = nh.subscribe<std_msgs::Float32MultiArray>("/lost_flag/param/cam",1,boost::bind(&dwe::state_cam_cb, &dwe_filter, _1));

    //==================== publisher =====================//
    // pub : check -- use for rosbag pla check
    // bias_pos, vel_z filtered/ est pos + est vel
    estimate_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/xt_real/pose", 1);
    estimate_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(dwe_filter.dataType_ + "/xt_real/vel", 1);

    // no bias_pos, vel no filtered/ est pos + diff vel
    estimate_pose_f_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/xt/pose", 1);
    estimate_vel_f_pub = nh.advertise<geometry_msgs::TwistStamped>(dwe_filter.dataType_ + "/xt/vel", 1);

    estimate_viconpose_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/viconxt_real/pose", 1);
    estimate_viconvel_pub = nh.advertise<geometry_msgs::TwistStamped>(dwe_filter.dataType_ + "/viconxt_real/vel", 1);

    // bias_pos + filtered / est pos + diff vel
    estimate_est_relpose_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/xt_f/pose", 1);
    estimate_est_relvel_pub = nh.advertise<geometry_msgs::TwistStamped>(dwe_filter.dataType_ + "/xt_f/vel", 1);   
    estimate_relpose_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/viconxt/pose", 1);
    estimate_relvel_pub = nh.advertise<geometry_msgs::TwistStamped>(dwe_filter.dataType_ + "/viconxt/vel", 1);

    estimate_relpose2origin_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/xt_ugv_f/pose", 1);
    estimate_viconrelpose2origin_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/viconxt_ugv_f/pose", 1);
    
    // pub : used data
    uesd_imu_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/used/imu", 1);       
    uesd_uwb_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/used/uwb", 1);     
    uesd_uwbvicon_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/used/uwbvicon", 1);     
    uesd_dt_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/used/dt", 1);      
    uesd_flow_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/used/flow", 1);     
    uesd_rel_flow_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/used/relative_flow", 1);       
    uesd_hgt_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/used/hgt", 1);      
    camera_A_pub = nh.advertise<geometry_msgs::PoseStamped>(dwe_filter.dataType_ + "/used/cameraA",1);
    
    // pub : just for data state check
    param_uwb_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/uwb", 1);   
    param_cam_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/cam", 1);   
    param_flow_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/flow", 1);    
    param_imu_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/imu", 1);    
    kappa_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/kappa", 1);   

    param_cov_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/cov", 1);   
    param_quality_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/quality", 1);    
    param_fail_pub = nh.advertise<std_msgs::Float32MultiArray>(dwe_filter.dataType_ + "/param/fail", 1);    

    // ================================ Main LOOP ========================= //
    while(ros::ok()){
 
        ros::spinOnce();

        // ensure imu msg is obtained
        if (!(dwe_filter.flag_init_imu_)){
            continue;
        }
        dwe_filter.update_data(); 

        // LOOP_num: current timestep
        ROS_INFO("----------ENTER ROS LOOP----------");
        ROS_INFO("LOOP: %d", dwe_filter.LOOP_num_);

        // show the data from uav
        ROS_INFO("DATA MHE_imu_: ux:%f uy:%f uz:%f",dwe_filter.MHE_imu_(0,dwe_filter.num_state_), 
            dwe_filter.MHE_imu_(1,dwe_filter.num_state_),dwe_filter.MHE_imu_(2,dwe_filter.num_state_));
        ROS_INFO("DATA MHE_uwb_: d:%f",dwe_filter.MHE_uwb_(0,dwe_filter.num_state_));
        ROS_INFO("DATA MHE_flow_: vx:%f vy:%f vz:%f hz:%f",
            dwe_filter.MHE_flow_(0,dwe_filter.num_state_), dwe_filter.MHE_flow_(1,dwe_filter.num_state_),
            dwe_filter.MHE_flow_(2,dwe_filter.num_state_), dwe_filter.MHE_height_(0,dwe_filter.num_state_));

        // get initial data xt_ until takeoff
        if(!dwe_filter.flag_init_ok_){
            // xt_real_pos & xt_real_vel (origin estimation)
            PubPose(dwe_filter.xt_real_, dwe_filter.num_state_ , estimate_pose_pub, 0);
            PubVelocity(dwe_filter.xt_real_, dwe_filter.num_state_, estimate_vel_pub, 3);
            // xt_real_f_pos & xt_real_f_vel (filtered estimation)
            PubPoseandOri(dwe_filter.xt_,dwe_filter.MHE_imu_att_,dwe_filter.num_state_, estimate_pose_f_pub, 0);
            PubVelocity(dwe_filter.xt_vel_,dwe_filter.num_state_, estimate_vel_f_pub, 0);

            // viconxt_real_pos & viconxt_real_vel
            PubPose(dwe_filter.VICON_PosUAV_, dwe_filter.num_state_, estimate_viconpose_pub, 0);
            PubVelocity(dwe_filter.VICON_VelUAV_, dwe_filter.num_state_, estimate_viconvel_pub, 0);
 
            PubPoseandOri(dwe_filter.xt_f_,dwe_filter.MHE_imu_att_,dwe_filter.num_state_, estimate_est_relpose_pub, 0);
            PubVelocity(dwe_filter.xt_f_,dwe_filter.num_state_, estimate_est_relvel_pub, 3);

            PubPose(dwe_filter.VICON_PosUAV2UWB_, 0, estimate_relpose_pub, 0);
            PubVelocity(dwe_filter.VICON_PosUAV2UWB_,0, estimate_relvel_pub, 3);

            PubPose(dwe_filter.xt_2origin_, dwe_filter.num_state_, estimate_relpose2origin_pub, 0);
            PubPose(dwe_filter.VICON_PosUAV2_fixedorigin_, 0, estimate_viconrelpose2origin_pub, 0);
            
            // cam_pos
            PubPose(dwe_filter.MHE_cam_, dwe_filter.num_state_, camera_A_pub, 0);
            PubPose(dwe_filter.MHE_flow_, dwe_filter.num_state_, uesd_flow_pub, 0);
            PubPose(dwe_filter.MHE_reletive_vel_, dwe_filter.num_state_, uesd_rel_flow_pub, 0);
            PubHgt(dwe_filter.MHE_reletive_height_, dwe_filter.num_state_, uesd_hgt_pub);
            // imu data
            PubPose(dwe_filter.MHE_imu_, dwe_filter.num_state_, uesd_imu_pub, 0);
            // uwb data
            Pubuwb(dwe_filter.MHE_uwb_, &dwe_filter, uesd_uwb_pub);
            Pubuwb(dwe_filter.VICON_uwb_, &dwe_filter, uesd_uwbvicon_pub);
            // dt & dt_solveonce
            Pubdt(dwe_filter.dt_,0,&dwe_filter);
            // uwb param
            Pubparamuwb(dwe_filter.paramUWB_,&dwe_filter);
            Pubparamcam(dwe_filter.paramCAM_,&dwe_filter);
            Pubparamimu(dwe_filter.paramIMU_,&dwe_filter);
            Pubparamflow(dwe_filter.paramFLOW_,dwe_filter.paramHGT_,&dwe_filter);
            Pubcov(dwe_filter.sensor.cov.imu_p,dwe_filter.sensor.cov.imu_v,dwe_filter.sensor.cov.flow,
            dwe_filter.sensor.cov.cam,dwe_filter.sensor.cov.hgt,dwe_filter.sensor.cov.uwb,param_cov_pub);
            Pubfail(dwe_filter.sensor.fail.imu,dwe_filter.sensor.fail.flow,
            dwe_filter.sensor.fail.cam,dwe_filter.sensor.fail.hgt,dwe_filter.sensor.fail.uwb,param_fail_pub);
            Pubquality(dwe_filter.sensor.quality.imu,dwe_filter.sensor.quality.flow,
            dwe_filter.sensor.quality.cam,dwe_filter.sensor.quality.hgt,dwe_filter.sensor.quality.uwb,dwe_filter.sensor.quality.base,param_quality_pub);
            Pubkappa(dwe_filter.MHE_kappa_,dwe_filter.fac);

            dwe_filter.LOOP_num_ = dwe_filter.LOOP_num_+1;
            dwe_filter.flag_init_imu_ = false; 
            loop_rate.sleep();
            continue;
        }
        
        // estimated initial value
        double time_start = ros::Time::now().toSec();  
        dwe_filter.solveonce();  
        double time_end = ros::Time::now().toSec();
        ROS_INFO("consumptiom dt: %.4lf seconds", time_end - time_start);

        // show the Initialized data x1_ and the Estimated data xt_
        // ROS_INFO("Estimated   data: x:%f y:%f z:%f",dwe_filter.xt_2origin_(0,dwe_filter.num_state_), 
        //     dwe_filter.xt_2origin_(1,dwe_filter.num_state_),dwe_filter.xt_2origin_(2,dwe_filter.num_state_));
        ROS_INFO("Estimated   data: x:%f y:%f z:%f",dwe_filter.xt_f_(0,dwe_filter.num_state_), 
            dwe_filter.xt_f_(1,dwe_filter.num_state_),dwe_filter.xt_f_(2,dwe_filter.num_state_));
        ROS_WARN("Error real  data: x:%f y:%f z:%f",
            (dwe_filter.xt_f_(0,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2UWB_(0,0)), 
            (dwe_filter.xt_f_(1,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2UWB_(1,0)),
            (dwe_filter.xt_f_(2,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2UWB_(2,0)));

        // ROS_WARN("Error real  data: x:%f y:%f z:%f",
        //     (dwe_filter.xt_2origin_(0,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2_fixedorigin_(0,0)), 
        //     (dwe_filter.xt_2origin_(1,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2_fixedorigin_(1,0)),
        //     (dwe_filter.xt_2origin_(2,dwe_filter.num_state_)-dwe_filter.VICON_PosUAV2_fixedorigin_(2,0)));

        // xt_real_pos & xt_real_vel
        PubPose(dwe_filter.xt_real_, dwe_filter.num_state_ , estimate_pose_pub, 0);
        PubVelocity(dwe_filter.xt_real_, dwe_filter.num_state_, estimate_vel_pub, 3);
        // xt_real_f_pos & xt_real_f_vel (filtered estimation)
        PubPoseandOri(dwe_filter.xt_,dwe_filter.MHE_imu_att_,dwe_filter.num_state_, estimate_pose_f_pub, 0);
        PubVelocity(dwe_filter.xt_vel_,dwe_filter.num_state_, estimate_vel_f_pub, 0);
        // viconxt_real_pos & viconxt_real_vel
        PubPose(dwe_filter.VICON_PosUAV_, dwe_filter.num_state_, estimate_viconpose_pub, 0);
        PubVelocity(dwe_filter.VICON_VelUAV_, dwe_filter.num_state_, estimate_viconvel_pub, 0);
        // xt_pos & xt_vel (origin estimation)
        PubPoseandOri(dwe_filter.xt_f_,dwe_filter.MHE_imu_att_,dwe_filter.num_state_, estimate_est_relpose_pub, 0);
        PubVelocity(dwe_filter.xt_f_,dwe_filter.num_state_, estimate_est_relvel_pub, 3);

        PubPose(dwe_filter.VICON_PosUAV2UWB_, 0, estimate_relpose_pub, 0);
        PubVelocity(dwe_filter.VICON_PosUAV2UWB_,0, estimate_relvel_pub, 3);

        PubPose(dwe_filter.xt_2origin_, dwe_filter.num_state_, estimate_relpose2origin_pub, 0);
        PubPose(dwe_filter.VICON_PosUAV2_fixedorigin_, 0, estimate_viconrelpose2origin_pub, 0);
        // cam_pos
        PubPose(dwe_filter.MHE_cam_, dwe_filter.num_state_, camera_A_pub, 0);
        PubPose(dwe_filter.MHE_flow_, dwe_filter.num_state_, uesd_flow_pub, 0);
        PubPose(dwe_filter.MHE_reletive_vel_, dwe_filter.num_state_, uesd_rel_flow_pub, 0);
        PubHgt(dwe_filter.MHE_reletive_height_, dwe_filter.num_state_, uesd_hgt_pub);
        // imu data
        PubPose(dwe_filter.MHE_imu_, dwe_filter.num_state_, uesd_imu_pub, 0);
        // uwb data
        Pubuwb(dwe_filter.MHE_uwb_, &dwe_filter, uesd_uwb_pub);
        Pubuwb(dwe_filter.VICON_uwb_, &dwe_filter, uesd_uwbvicon_pub);
        // dt & dt_solveonce
        Pubdt(dwe_filter.dt_,time_end - time_start,&dwe_filter);
        // uwb param
        Pubparamuwb(dwe_filter.paramUWB_,&dwe_filter);
        Pubparamcam(dwe_filter.paramCAM_,&dwe_filter);
        Pubparamimu(dwe_filter.paramIMU_,&dwe_filter);
        Pubparamflow(dwe_filter.paramFLOW_,dwe_filter.paramHGT_,&dwe_filter);
        Pubcov(dwe_filter.sensor.cov.imu_p,dwe_filter.sensor.cov.imu_v,dwe_filter.sensor.cov.flow,
        dwe_filter.sensor.cov.cam,dwe_filter.sensor.cov.hgt,dwe_filter.sensor.cov.uwb,param_cov_pub);
        Pubfail(dwe_filter.sensor.fail.imu,dwe_filter.sensor.fail.flow,
        dwe_filter.sensor.fail.cam,dwe_filter.sensor.fail.hgt,dwe_filter.sensor.fail.uwb,param_fail_pub);
        Pubquality(dwe_filter.sensor.quality.imu,dwe_filter.sensor.quality.flow,
        dwe_filter.sensor.quality.cam,dwe_filter.sensor.quality.hgt,dwe_filter.sensor.quality.uwb,dwe_filter.sensor.quality.base,param_quality_pub);
        Pubkappa(dwe_filter.MHE_kappa_,dwe_filter.fac);

        dwe_filter.LOOP_num_ = dwe_filter.LOOP_num_+1;
        dwe_filter.flag_init_imu_ = false; 
        loop_rate.sleep();

    }
   
    ROS_INFO("----------FINISH ROS LOOP----------");
    return 0;
}
