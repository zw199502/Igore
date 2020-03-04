#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <queue> // std::queue
#include <deque>          // std::deque
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
#include "Iir.h" // iir filter library
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>
#include <armadillo> // Linear algebra library
#include "kalman/ekfilter.hpp"




class igor_knee_control
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_pose;
    //geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::TwistWithCovariance igor_twist;
    geometry_msgs::Point igor_position;
    geometry_msgs::Point CoG_Position;
    geometry_msgs::Vector3 igor_linear_vel;
    //geometry_msgs::TransformStamped transformStamped;
    //visualization_msgs::Marker marker;


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2Listener{tfBuffer};
    
    
    
    


    //std_msgs::Header  imu_header; // Header type variable 
    float igor_pos_x = 0;
    float igor_pos_y = 0;
    float igor_vel_x = 0;
    float igor_vel_y = 0;
    //float robot_center_pos_x = 0;
    //float robot_center_pos_y = 0;
    //float robot_center_pos_z = 0;
    float igor_center_position = 0;
    float igor_center_vel = 0;
    
    
    //ros::Duration dt{0}; //sampling time
    //ros::Duration dt2{0}; 

    double roll, pitch, yaw = 0.0;
    std_msgs::Float64 trq_r;
    std_msgs::Float64 trq_l;
    std_msgs::Float64 knee_ref;
    std_msgs::Float64 hip_ref;
    std::vector<double> joint_pos; // Array
    std::vector<double> joint_vel; // Array
    double L_knee_pos;
    double L_knee_vel;
    double R_knee_pos;
    double R_knee_vel;
    float L = 0;

    float CoG_angle = 0;
    //double CoG_angle2 = 0;
    float CoG_angle_filtered =0;
    //float CoG_angle3 = 0;
    //float CoG_last_angle = 0;
    //float CoG_pitch_vel = 0;
    //float CoG_last_pitch_vel = 0;
    //float CoG_last_angle2 = 0;
    float CoG_angle_vel = 0;
    //float CoG_last_pitch_vel2 = 0;

    //ros::Time now;
    //ros::Time last_time;
    //ros::Time now2;
    //ros::Time last_time2;
    //ros::Time time_strt;
    //ros::Duration time_finsh;




    
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

    // float pitch_filt_out = 0;
    // float pitch_filt_in = 0;
    // float last_pitch_filt_out = 0.0;
    // float last_pitch_filt_in = 0.0;




    //geometry_msgs::Twist vel_cmnd;
    geometry_msgs::Point ref_origin;
    //geometry_msgs::Point p;
    
    tf::Quaternion quat;

    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_center_imu; // creating ROS subscriber
    ros::Subscriber sub_joint_states; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    ros::Subscriber sub_CoG; // creating ROS subscriber
    

    
    ros::Publisher  state_pub; // creating ROS publisher
    ros::Publisher  state_pub2; // creating ROS publisher
    ros::Publisher  Lwheel_pub; // creating ROS publisher
    ros::Publisher  Rwheel_pub; // creating ROS publisher
    ros::Publisher  Lknee_pub; // creating ROS publisher
    ros::Publisher  Rknee_pub; // creating ROS publisher
    ros::Publisher  Lhip_pub; // creating ROS publisher
    ros::Publisher  Rhip_pub; // creating ROS publisher
    
    //ros::Publisher  marker_pub;
    //ros::Publisher  pose_pub;  

    void center_imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void statePub (geometry_msgs::Vector3 x);
    void statePub2 (geometry_msgs::Vector3 x);
    void lqr_controller(Eigen::VectorXf eig_vec);
    void CT_controller(Eigen::VectorXf eig_vec);
    void ref_update();

    Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
    Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    //Eigen::MatrixXf k_k = Eigen::MatrixXf(1,8); // declaring 1X6 Eigen matrix of datatype float
    Eigen::VectorXf igor_state = Eigen::VectorXf(6); // declaring 6x1 Eigen vector of datatype float;
    Eigen::VectorXf ref_state = Eigen::VectorXf(6);
    Eigen::Vector3d robot_center_pos;
    Eigen::Vector3d CoM_pos;
    Eigen::Vector3d CoM_vec;
    Eigen::Vector3d unit_vec;
    //Eigen::MatrixXd rot = Eigen::MatrixXd(3,3);

    Eigen::MatrixXd M_h = Eigen::MatrixXd(3,3);
    Eigen::Vector3d H_h;
    Eigen::MatrixXd V_h = Eigen::MatrixXd(3,3);
    Eigen::Vector3d G_h;
    Eigen::MatrixXd E_h_inv = Eigen::MatrixXd(2,3);
    Eigen::Vector3d Ep;
    Eigen::Vector3d Ev;
    Eigen::Vector3d velocities;
    Eigen::MatrixXd Kp = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd Kv = Eigen::MatrixXd(3,3);
    float Kp1 = 1.5; // Linear postion gain
    float Kp2 = 30; // Yaw gain
    float Kp3 = 85; // Pitch gain
    float Kv1 = 0.75; // Linear velocity gain
    float Kv2 = 10; // Yaw speed gain
    float Kv3 = 10; // Pitch speed gain
    Eigen::Vector3d feedbck;
    Eigen::Vector2d output_trq;

    //Eigen::MatrixXf transf_matrix = Eigen::MatrixXf(4,4);
    tf::Vector3 CoM_tf;
    tf::Vector3 unit_tf;
    tf::Vector3 rot_axis{0,1,0};

    ros::ServiceClient client;
    std_srvs::Empty srv;

public:

    igor_knee_control(ros::NodeHandle* nodehandle); // constructor
    ~igor_knee_control(); // destructor

    float pitch_vel_y = 0;
    float yaw_vel_z = 0;
    float M_pi = 3.1415;
    //float freq = 0;
    geometry_msgs::Vector3 state_vec;
    geometry_msgs::Vector3 state_vec2;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
    //geometry_msgs::Vector3 rpy; // orientation in roll, pitch, and yaw

    // const int order = 4; // 4th order (=2 biquads)
    //Iir::Butterworth::LowPass<1> bwf;
    //const float samplingrate = 500; // Hz
    //const float cutoff_frequency = 4; // Hz
    
    // float y1 = 0;
    // float y2 = 0;
    // float y3 = 0;
    // float y4 = 0;

    // Window size is 2*m+1
    const int m = 15;
    const int m2 = 5;
    //const int m3 = 30;
    //const int m4 = 20;
    // Polynomial Order
    const int n = 0;
    const int n2 = 1;
    //const int n3 = 1;
    //const int n4 = 0;
    // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
    // Points are defined in range [-m;m]
    const int t = m;
    const int t2 = m2;
    //const int t3 = m3;
    //const int t4 = m4;
    // Derivate? 0: no derivation, 1: first derivative...
    const int d = 0;
    double result;
    gram_sg::SavitzkyGolayFilterConfig sg_conf{m,t,n,d,0.002}; // filter configuration
    gram_sg::SavitzkyGolayFilterConfig sg_conf2{m2,t2,n2,1,1}; // filter configuration
    //gram_sg::SavitzkyGolayFilterConfig sg_conf3{m3,t3,n3,1,1}; // filter configuration
    //gram_sg::SavitzkyGolayFilterConfig sg_conf4{m4,t4,n4,d,0.002}; // filter configuration
    gram_sg::SavitzkyGolayFilter f1{sg_conf}, f2{sg_conf2};
     
    boost::circular_buffer<double> my_data1 {boost::circular_buffer<double>((2*m+1),0)};
    //boost::circular_buffer<double> my_data5 {boost::circular_buffer<double>((2*m4+1),0)};
    //boost::circular_buffer<double> my_data6 {boost::circular_buffer<double>((2*m4+1),0)};
    
    boost::circular_buffer<double> my_data2 {boost::circular_buffer<double>((2*m2+1),-0.033)}; // Initialize with -0.033
    //boost::circular_buffer<double> my_data3 {boost::circular_buffer<double>((2*m3+1),-0.033)};
    //boost::circular_buffer<double> my_data4 {boost::circular_buffer<double>((2*m3+1),-0.033)};


}; //end of class