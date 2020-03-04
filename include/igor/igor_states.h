#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
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
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>



class igor_states
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_pose;
    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::TwistWithCovariance igor_twist;
    geometry_msgs::Point igor_position;
    geometry_msgs::Point CoG_Position;
    geometry_msgs::Vector3 igor_linear_vel;
    geometry_msgs::TransformStamped transformStamped;
    visualization_msgs::Marker marker;


    tf2_ros::Buffer tfBuffer{ros::Duration(10)};
    tf2_ros::TransformListener tf2Listener{tfBuffer};
    
    
    


    //std_msgs::Header  imu_header; // Header type variable 
    float igor_pos_x = 0;
    float igor_pos_y = 0;
    float robot_center_pos_x = 0;
    float robot_center_pos_y = 0;
    float robot_center_pos_z = 0;
    float igor_center_position = 0;
    float igor_center_vel = 0;
    
    float igor_vel_x = 0;
    float igor_vel_y = 0;
    ros::Duration dt{0}; //sampling time
    ros::Duration dt2{0}; 

    double roll, pitch, yaw = 0.0;
    std_msgs::Float64 knee_ref;
    std::vector<double> joint_pos; // Array
    std::vector<double> joint_vel; // Array
    double L_knee_pos;
    double L_knee_vel;
    double R_knee_pos;
    double R_knee_vel;
    float CoG_angle = 0;
    float CoG_angle2 = 0;
    float CoG_angle3 = 0;
    float CoG_last_angle = 0;
    float CoG_pitch_vel = 0;
    float CoG_last_pitch_vel = 0;
    float CoG_last_angle2 = 0;
    float CoG_pitch_vel2 = 0;
    float CoG_last_pitch_vel2 = 0;
    ros::Time now;
    ros::Time last_time;
    ros::Time now2;
    ros::Time last_time2;
    ros::Time time_strt;
    ros::Duration time_finsh;
    //float pitch_vel_y = 0;
    float yaw_vel_z = 0;
    float M_pi = 3.1415;
    float freq = 0;
    geometry_msgs::Vector3 state_vec;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
    //geometry_msgs::Vector3 rpy; // orientation in roll, pitch, and yaw



    
    float filt1 = 0.02817; // LPF const.
    float filt2 = 0.9718; // LPF const.
    
    // float filt_out_r = 0.0;
    // float filt_in_r = 0.0;
    // float last_filt_out_r = 0.0;
    // float last_filt_in_r = 0.0;
    // float filt_out_l = 0.0;
    // float filt_in_l = 0.0;
    // float last_filt_out_l = 0.0;
    // float last_filt_in_l = 0.0;
    float vel_filt_out = 0;
    float vel_filt_in = 0;
    float last_vel_filt_out = 0.0;
    float last_vel_filt_in = 0.0;



    geometry_msgs::Twist vel_cmnd;
    geometry_msgs::Point ref_origin;
    geometry_msgs::Point p;
    
    tf::Quaternion quat;

    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_center_imu; // creating ROS subscriber
    ros::Subscriber sub_joint_states; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    ros::Subscriber sub_CoG; // creating ROS subscriber
    


    void center_imu_callback(const sensor_msgs::Imu::ConstPtr &msg1);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg2);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg3);
    void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg4);
  
    Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
    Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    //Eigen::MatrixXf k_k = Eigen::MatrixXf(1,8); // declaring 1X6 Eigen matrix of datatype float
    
    Eigen::VectorXf ref_state = Eigen::VectorXf(6);
    Eigen::Vector3d robot_center_pos;
    Eigen::Vector3d CoM_pos;
    Eigen::Vector3d CoM_vec;
    Eigen::Vector3d unit_vec;
    Eigen::MatrixXd rot = Eigen::MatrixXd(3,3);
    tf::Vector3 CoM_tf;
    tf::Vector3 unit_tf;
    tf::Vector3 rot_axis{0,1,0};

    //ros::ServiceClient client;
    //std_srvs::Empty srv;

public:

    igor_states(ros::NodeHandle* nodehandle); // constructor
    ~igor_states(); // destructor
    //Eigen::VectorXf igor_state = Eigen::VectorXf(6); // declaring 6x1 Eigen vector of datatype float;
}; //end of class

extern Eigen::VectorXf igor_state = Eigen::VectorXf(6);