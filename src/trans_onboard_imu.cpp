// author:xps
// 2023-03-02
// if no msg from cam use vicon for pos

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <iostream>
#include <sensor_msgs/Imu.h>

int new_msg_flag;
geometry_msgs::PoseStamped pos;
geometry_msgs::TwistStamped vel;
sensor_msgs::Imu ori;
geometry_msgs::PoseStamped pos_to_pub;
nav_msgs::Odometry odom_to_pub;

bool have_pos = false;
bool have_vel = false;
bool have_ori = false;

class QuaternionAverager
{
public:
    QuaternionAverager() : count(0) {}

    void addQuaternion(const Eigen::Quaterniond &q)
    {
        std::cout << "q: " << q.norm() << std::endl;
        if (count == 0)
        {
            average = q;
            std::cout << "average: " << average.norm() << std::endl;
        }
        else
        {
            double t = 1.0 / (count + 1);
            average = average.slerp(t, q);
            std::cout << "average: " << average.norm() << std::endl;
        }
        count++;
    }

    Eigen::Quaterniond getAverage() const
    {
        return average;
    }

private:
    Eigen::Quaterniond average;
    int count;
};

void posCallback(const geometry_msgs::PoseStamped &msg)
{
    pos = msg;
    have_pos = true;
}

void velCallback(const geometry_msgs::TwistStamped &msg)
{
    vel = msg;
    have_vel = true;
}

void oriCallback(const sensor_msgs::Imu &msg)
{
    ori = msg;
    have_ori = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trans");
    ros::NodeHandle nh;
    ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>(
        "/ctrl_pose/pose", 10);
    ros::Publisher pubOdom = nh.advertise<nav_msgs::Odometry>(
        "/ctrl_pose/odom", 10);
    ros::Subscriber subPos = nh.subscribe("/vicon/uav/pose", 10, posCallback);
    ros::Subscriber subVel = nh.subscribe("/vicon/uav/vel", 10, velCallback);
    ros::Subscriber subOri = nh.subscribe("/IMU_data", 10, oriCallback);
    ros::Rate rate(100);

    // int cnt = 0;
    // QuaternionAverager averager;

    // while (ros::ok() && cnt < 50)
    // {
    //     ros::spinOnce();
    //     if (have_pos && have_vel)
    //     {
    //         averager.addQuaternion(Eigen::Quaterniond(pos.pose.orientation.w,
    //                                                   pos.pose.orientation.x,
    //                                                   pos.pose.orientation.y,
    //                                                   pos.pose.orientation.z));
    //         cnt++;
    //         ROS_INFO("Inilizing orientation...");
    //     }
    //     rate.sleep();
    // }
    // Eigen::Quaterniond q_init = averager.getAverage();
    // ROS_INFO("Finish inilizing orientation! Average Quaternion: %f %f %f %f",
    //          q_init.w(), q_init.x(), q_init.y(), q_init.z());

    while (ros::ok())
    {
        ros::spinOnce();

        pos_to_pub.header = pos.header;
        odom_to_pub.header = pos.header;

        pos_to_pub.pose.position = pos.pose.position;
        pos_to_pub.pose.orientation = ori.orientation;

        // Eigen::Quaterniond q(pos.pose.orientation.w,
        //                      pos.pose.orientation.x,
        //                      pos.pose.orientation.y,
        //                      pos.pose.orientation.z);
        // q = q * q_init.inverse();
        // pos_to_pub.pose.orientation.w = q.w();
        // pos_to_pub.pose.orientation.x = q.x();
        // pos_to_pub.pose.orientation.y = q.y();
        // pos_to_pub.pose.orientation.z = q.z();

        odom_to_pub.pose.pose = pos_to_pub.pose;
        odom_to_pub.twist.twist = vel.twist;

        pubPose.publish(pos_to_pub);
        pubOdom.publish(odom_to_pub);
        rate.sleep();
    }

    return 0;
}
