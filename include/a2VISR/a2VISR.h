#ifndef _A2VISR_H
#define _A2VISR_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <fstream>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>

#include "math_tools.h"

using namespace Eigen;
using namespace std;

class dwe{
    private:
        // A and B can be a private member 
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 3> B;
        Eigen::Matrix<double, 6, 6> AA;

        // param for MAP sliding window
		int delta_, lopt_, order_, k_, rate_;
        int vicon_flag_, param_flag_;
        int init_num_;

        // param for dwe_param
		double paramPp_, paramPv_, paramQp_, paramQv_;
        Eigen::Matrix<double, 1, 3> kappa_;

        // param for msg_bias
		double bias_imu_;
        Eigen::Matrix<double, 1, 3> bias_uwb_;
        Eigen::Matrix<double, 1, 3> bias_pos_;
        Eigen::Matrix<double, 1, 3> bias_marker_;   
        Eigen::Matrix<double, 1, 3> bias_marker_ugv_;   
        float bias_d_;
        Eigen::MatrixXd bias_cam_;

        double uwb_parm; // 初始化为paramR_

        // param for cb
        bool cam_data_received_;
        vector<geometry_msgs::TransformStamped> cam_cbs_;
        geometry_msgs::PoseStamped uwb_pos_msg; 
        mavros_msgs::OpticalFlowRad flow_msg;
        nlink_parser::LinktrackNodeframe3 uwb_msg;
        geometry_msgs::PoseStamped VICON_posUAV_msg;
        geometry_msgs::TwistStamped VICON_velUWB_msg;
        geometry_msgs::TwistStamped VICON_velUAV_msg;
        sensor_msgs::Imu imu_msg;
        std_msgs::Float32MultiArray state_cam_msg;

    public:
        //默认构造函数
        dwe(){}
        // 构造函数
        dwe(ros::NodeHandle& nh);
        //默认析构函数
        ~dwe();

        string dataType_;

        // param for msg_sensor
		int uwbnum_, useduwbnum_;
        int camnum_, usedcamnum_;
        double cam_fail_state = 2.0;
        
        // [无人机中心(IMU)] 在 [VICON坐标系] 下的位置
        Vector3d pos_uav_init_;
        // [UWB0] 在 [VICON坐标系] 下的位置
        Vector3d pos_uwb01_init_;
        // [无人机中心(IMU)] 在 [UWB0坐标系] 下的位置
        Vector3d pos_uav2uwb01_init_;
        // [UWB锚点0123] 在 [UWB0坐标系] 下的位置
        // Eigen::MatrixXd pos_uwb_init_;

        // param matrix
        MatrixXd paramX0_, paramIMU_, paramUWB_;
        MatrixXd paramCAM_, paramFLOW_, paramHGT_;
        Eigen::Matrix<double, 1, 3> MHE_kappa_;
        // MatrixXd MHE_kappa_;

        int count_j_;
        int LOOP_num_;
        int startNum_;
        int num_state_;

        ros::Time lost_initTime_ = ros::Time(0);
        ros::Time currentTime_;
        ros::Time previousTime_ = ros::Time(0);
        int flow_quality_;
        float dt_ = 0.0;

        MatrixXd current_cam_stamp;
        MatrixXd last_cam_stamp;
        MatrixXd last_update_time;

        //ensure data input
        bool flag_init_imu_= false; 
        bool check_for_init_= false; 
        bool get_init_imu= false; 
        bool flag_init_ok_ = false; 
        bool hasTakenOff_ = false; 
        bool hasStartEst_ = false;
        double scaling_factor;

        //param in estimation
        MatrixXd MHE_height_, MHE_reletive_height_, MHE_hgt_residual_;
        MatrixXd MHE_flow_, MHE_reletive_vel_, MHE_flow_residual_;
        MatrixXd MHE_imu_, MHE_imu_residual_;
        MatrixXd MHE_imu_att_;
        MatrixXd MHE_uwb_, MHE_uwb_residual_;
        MatrixXd MHE_uwb_bias_;
        MatrixXd xt_2origin_,MHE_ugv2origin_;
        // MatrixXd MHE_uwb_sensor_;
        MatrixXd MHE_cam_, MHE_cam_residual_;
        Eigen::Matrix3d rotation_matrix;

        MatrixXd VICON_PosUAV_;
        MatrixXd VICON_PosUWB_;
        MatrixXd VICON_VelUAV_, VICON_VelUWB_;
        MatrixXd VICON_uwb_;
        MatrixXd VICON_uwb_ini;
        MatrixXd VICON_PosUAV2_fixedorigin_;
        MatrixXd VICON_PosUAV2UWB_;

        MatrixXd cov_pre_xt_new;
        
        MatrixXd deltaTime_;
        MatrixXd xt_;  // recommend: stl/boost list
        MatrixXd xt_real_f_;  // recommend: stl/boost list
        MatrixXd xt_f_;  // recommend: stl/boost list
        MatrixXd xt_vel_;  // recommend: stl/boost list
        MatrixXd xt_real_;
        MatrixXd x1_;  // initialize with nonzero numbers, e.g., x=y=z=uwb.dis/sqrt(3);

        MatrixXd Ex,Eu,W,t_matrix,ur,xtt;
        MatrixXd ar,xxt_new,xxe,Ex_new;

        geometry_msgs::TransformStamped EstimationStamped;
        Eigen::Vector3d t_base_coopestimation = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_base_coopestimation = Eigen::Quaterniond::Identity();

        // param
        // 故障标记结构体
        struct FailureFlags {
            int cam = 1;    // 相机故障标记
            int uwb = 1;    // UWB故障标记
            int hgt = 1;    // 高度计故障标记
            int flow = 1;   // 光流故障标记
            int imu = 1;    // IMU故障标记
        };

        // 质量参数结构体
        struct QualityParams {
            Vector3d cam;    // 相机三维质量
            Vector3d flow;   // 光流三维质量
            Vector3d imu;    // IMU三维质量
            Vector3d imu_p;    // IMU三维质量
            Vector3d imu_v;    // IMU三维质量
            double uwb;                        // UWB质量
            double hgt;                        // 高度计质量
            double base;                       // 基础质量参数
            double sum;                       // 基础质量参数
            // double sumR;                       // 基础质量参数
            // double sumP;                       // 基础质量参数
            // double sumV;                       // 基础质量参数
        };

        // 主参数结构体
        struct SensorParams {
            QualityParams quality;        // 质量相关参数
            FailureFlags fail;            // 故障标记
            QualityParams weight;          // 权重参数
            QualityParams factor;          // 因子参数
            QualityParams cov;  // 协方差参数
            QualityParams param;
            QualityParams cov_per;  // 协方差参数
        };
        SensorParams sensor;
        SensorParams sum_cov;
        double fac ;

        double residual_threshold = 1e-6;

    public:
        // define the input data form uav
        void uwb3_cb(const nlink_parser::LinktrackNodeframe3::ConstPtr &msg);
        void imu_new_cb(const sensor_msgs::Imu::ConstPtr &msg);
        void camA_cb(const geometry_msgs::TransformStamped::ConstPtr &msg);
        void flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);

        // define the input data form vicon
        void pos_uav_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void pos_uwba0_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        
        void state_cam_cb(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void vel_uav_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void vel_uwb_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

        void Initialdataxt();
        void InitialWithParam();
        void solveonce();
        void update_data();
        void initializeEstimation();
        // void processUWBData(const nlink_parser::LinktrackNodeframe3 &uwb_msg, MatrixXd MHE_uwb_X_);
};

int data_loss_detection(const Eigen::MatrixXd& MHE_data_, double loss_epsilon);
double sigmoid(double x, double k, double x0);
Eigen::MatrixXd sigmoid_eigen(const Eigen::MatrixXd& x, 
                       double k, 
                       double x0);

#endif
