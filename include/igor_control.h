#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <vector>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <Eigen/Dense>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class igor_control
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_odom;
    geometry_msgs::Point igor_position;
    //std_msgs::Header  imu_header; // Header type variable 
    float igor_pos_x;
    float igor_last_pos = 0.0;
    float igor_vel_x;
    float dt = 0.001; //sampling time of IMU (1000Hz)
    float last_ang_vel = 0.0;
    double roll, pitch, yaw;
    std_msgs::Float64 trq;
    float sp1 = -0.0345; // position set point for PID
    float sp2 = 0.03401*70; // position set point for Poly PD
    float error;
    float last_err = 0;
    float error_d;
    float error_i = 0;
    float kp = 15; // PID gains
    float kd = 2;
    float ki = 5;

    float k_p = -73.0519; // Poly PD gains
    float k_d = -19.4697;
    float PD_out = 0;
    float sys_in = 0;
    
    float filt1 = 0.02817; // LPF const.
    float filt2 = 0.9718; // LPF const.
    
    float filt_out = 0.0;
    float filt_in = 0.0;
    float last_filt_out = 0.0;
    float last_filt_in = 0.0;
    
    float vel_filt_out = 0;
    float vel_filt_in = 0;
    float last_vel_filt_out = 0.0;
    float last_vel_filt_in = 0.0;
    //float Nbar = 1000;
    float offset = 0.42;

    float accl = 0;
    float omga = 0;
    float whl_Inertia = 0.001806;
    float lin_vel = 0;
    float wheel_rad = 0.1016;
    geometry_msgs::Twist vel_cmnd;
    
    tf::Quaternion quat;
    //tf2::Quaternion quat;

    ros::NodeHandle nh; // creating ROS NodeHandle
    ros::Subscriber sub_imu; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    message_filters::Subscriber<sensor_msgs::Imu> sync_imu;
    message_filters::Subscriber<nav_msgs::Odometry> sync_odom;
    typedef message_filters::TimeSynchronizer<sensor_msgs::Imu, nav_msgs::Odometry> sync;
    boost::shared_ptr<sync> sync_;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // boost::shared_ptr<Sync> sync_;
    
    ros::Publisher  pub; // creating ROS publisher
    ros::Publisher  cmd_pub1; // creating ROS publisher
    ros::Publisher  cmd_pub2; // creating ROS publisher

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void sync_callback(const sensor_msgs::Imu::ConstPtr &msg1, const nav_msgs::Odometry::ConstPtr &msg2);
    void statePub (geometry_msgs::Vector3 x);
    void sf_controller(Eigen::Vector4f eig_vec);
    void PID_controller();
    void Poly_PID();
    Eigen::Vector3f eig_vec; // declaring 3X1 Eigen vector of datatype float
    Eigen::MatrixXf k_vec = Eigen::MatrixXf(1,4); // declaring 1X4 Eigen matrix of datatype float
    Eigen::MatrixXf k_lqr = Eigen::MatrixXf(1,4); // declaring 1X4 Eigen matrix of datatype float
    Eigen::Vector4f igor_state;
    Eigen::Vector4f ref_state;

    ros::ServiceClient client;
    std_srvs::Empty srv;

public:

    igor_control(); // constructor
    ~igor_control(); // destructor

    float ang_accl_y;
    float ang_vel_y;
    geometry_msgs::Vector3 state_vec;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
    //geometry_msgs::Vector3 rpy; // orientation in roll, pitch, and yaw


}; //end of class