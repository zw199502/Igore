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
#include "geometry_msgs/TwistWithCovariance.h"
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

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>



class igor_3d_control
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_pose;
    geometry_msgs::TwistWithCovariance igor_twist;
    geometry_msgs::Point igor_position;
    geometry_msgs::Vector3 igor_linear_vel;
    tf::Transform transform;
    tf::Quaternion q;
    
    //std_msgs::Header  imu_header; // Header type variable 
    float igor_pos_x;
    float igor_pos_y;
    float pos_y_offset = 0.17773;
    float igor_center_position;
    float igor_center_vel;
    float igor_last_pos_x = 0.0;
    float igor_last_pos_y = 0.0;
    float igor_last_center = 0.0;
    float igor_vel_x;
    float igor_vel_y;
    float dt = 0.001; //sampling time of IMU (1000Hz)
    float last_pitch_vel = 0.0;
    float last_yaw_vel = 0.0;
    double roll = 0, pitch = 0, yaw = 0;
    std_msgs::Float64 trq_r;
    std_msgs::Float64 trq_l;




    
    float filt1 = 0.02817; // LPF const.
    float filt2 = 0.9718; // LPF const.
    
    float filt_out_r = 0.0;
    float filt_in_r = 0.0;
    float last_filt_out_r = 0.0;
    float last_filt_in_r = 0.0;
    float filt_out_l = 0.0;
    float filt_in_l = 0.0;
    float last_filt_out_l = 0.0;
    float last_filt_in_l = 0.0;
    float vel_filt_out = 0;
    float vel_filt_in = 0;
    float last_vel_filt_out = 0.0;
    float last_vel_filt_in = 0.0;
    //float Nbar = 1000;



    geometry_msgs::Twist vel_cmnd;
    
    tf::Quaternion quat;

    ros::NodeHandle nh; // creating ROS NodeHandle
    ros::Subscriber sub_imu; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    message_filters::Subscriber<sensor_msgs::Imu> sync_imu;
    message_filters::Subscriber<nav_msgs::Odometry> sync_odom;
    typedef message_filters::TimeSynchronizer<sensor_msgs::Imu, nav_msgs::Odometry> sync;
    boost::shared_ptr<sync> sync_;

    
    ros::Publisher  pub; // creating ROS publisher
    ros::Publisher  cmd_pub1; // creating ROS publisher
    ros::Publisher  cmd_pub2; // creating ROS publisher

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void statePub (geometry_msgs::Vector3 x);
    void lqr_controller(Eigen::VectorXf eig_vec);
    void ref_update();

    Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
    Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::VectorXf igor_state = Eigen::VectorXf(6); // declaring 6x1 Eigen vector of datatype float;
    Eigen::VectorXf ref_state = Eigen::VectorXf(6);;

    ros::ServiceClient client;
    std_srvs::Empty srv;

public:

    igor_3d_control(); // constructor
    ~igor_3d_control(); // destructor

    //float pitch_accl_y;
    float pitch_vel_y;
    //float yaw_accl_z;
    float yaw_vel_z;
    float M_pi = 3.1415;
    float freq = 0;
    geometry_msgs::Vector3 state_vec;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
    //geometry_msgs::Vector3 rpy; // orientation in roll, pitch, and yaw


}; //end of class