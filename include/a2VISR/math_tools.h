#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <cstdlib>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <queue>
#include <vector>
#include <chrono>
#include <sstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h> 

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/core/eigen.hpp>
// #include <opencv2/core/utility.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/video/tracking.hpp>
// #include <cv_bridge/cv_bridge.h>

using namespace std;

const double pi = 3.1415926;
// param in sliding window
int M_sg_size_ = 9;
int N_sg_poly_ = 3;

geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
Eigen::Vector3d  quaternion2euler(float x, float y, float z, float w);
Eigen::MatrixXd quaternion2mat(Eigen::MatrixXd qua);
float get_lines_arctan(float line_1_k, float line_2_k, int aaa);
double vectorNorm2D(Eigen::Vector2d& vec) ;
double vectorDotProduct(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2);
double vectorAngle(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2, int method);
bool checkRotationDirection(Eigen::Vector2d& init, Eigen::Vector2d& final);

//Eigen2TF
tf::Quaternion EigenQuaterniondToTFQuaternion(Eigen::Quaterniond q_EIGEN) ;
tf::Vector3 EigenVector3dToTFVector3(Eigen::Vector3d t);

//TF2Eigen
Eigen::Quaterniond TFQuaternionToEigenQuaterniond(tf::Quaternion q_TF);
Eigen::Vector3d TFVector3ToEigenVector3d(tf::Vector3 t);

// //filter
Eigen::MatrixXd sgolayfilt(Eigen::MatrixXd data);
Eigen::MatrixXd meanFilter(Eigen::MatrixXd data);
Eigen::MatrixXd meanFilterWithSlidingWindow(Eigen::MatrixXd data, int windowSize);

#endif