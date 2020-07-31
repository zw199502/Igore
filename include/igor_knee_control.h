#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <queue> // std::queue
#include <deque> // std::deque
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>
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
#include "rosgraph_msgs/Clock.h"
//#include "Iir.h" // iir filter library
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>
#include <armadillo> // Linear algebra library
//#include "kalman/ekfilter.hpp" // Kalman filter library




class igor_knee_control
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_pose;
    geometry_msgs::TwistWithCovariance igor_twist;
    geometry_msgs::Point igor_position;
    geometry_msgs::Point CoG_Position;
    geometry_msgs::Vector3 igor_linear_vel;
    
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped leftLegTransformStamped;
    geometry_msgs::TransformStamped rightLegTransformStamped;


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2Listener{tfBuffer};

    tf2_ros::Buffer leftLegTfBuffer;
    tf2_ros::TransformListener leftLegTfListener{leftLegTfBuffer};
    tf2_ros::Buffer rightLegTfBuffer;
    tf2_ros::TransformListener rightLegTfListener{rightLegTfBuffer};
    
    
    
    
    float igor_pos_x = 0;
    float igor_pos_y = 0;
    float igor_vel_x = 0;
    float igor_vel_y = 0;
    float igor_center_position = 0;
    float igor_center_vel = 0;

    float lqr_right_trq = 0;
    float lqr_left_trq = 0;
    
    
    //ros::Duration dt{0}; //sampling time

    double roll, pitch, yaw = 0.0;
    std_msgs::Float64 lqr_trq_r;
    std_msgs::Float64 lqr_trq_l;
    std_msgs::Float64 CT_trq_r;
    std_msgs::Float64 CT_trq_l;
    std_msgs::Float64 trq_r;
    std_msgs::Float64 trq_l;
    std_msgs::Float64 knee_ref;
    std_msgs::Float64 hip_ref;
    std_msgs::Float32MultiArray plot_vector;

    float L = 0;

    float CoG_angle, leanAngle, CoM_height = 0;

    float CoG_angle_filtered = 0;
    //float CoG_angle_vel = 0;

    float CoM_acc_x;
    float CoM_acc_y;
    float CoM_acc_z;
    float ground_level = 0;
    float alpha = 0;



    
    //float filt1 = 0.02817; // LPF const.
    //float filt2 = 0.9718; // LPF const.
    
  
    //float vel_filt_out = 0;
    //float vel_filt_in = 0;
    //float last_vel_filt_out = 0.0;
    //float last_vel_filt_in = 0.0;

   




    
    //geometry_msgs::Point ref_origin;
    
    
    tf::Quaternion quat;

    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_body_imu; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    ros::Subscriber sub_CoG; // creating ROS subscriber
    ros::Subscriber clk_subscriber; // creating ROS subscriber
    

    
    ros::Publisher  Lwheel_pub; // creating ROS publisher
    ros::Publisher  Rwheel_pub; // creating ROS publisher
    ros::Publisher  Lknee_pub; // creating ROS publisher
    ros::Publisher  Rknee_pub; // creating ROS publisher
    ros::Publisher  Lhip_pub; // creating ROS publisher
    ros::Publisher  Rhip_pub; // creating ROS publisher
    ros::Publisher  zram_pub; // creating ROS publisher
    ros::Publisher  f_pub; // creating ROS publisher
    ros::Publisher  plot_publisher;
    


    void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    //void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void lqr_controller(Eigen::VectorXf eig_vec);
    void CT_controller(Eigen::VectorXf eig_vec);
    void ff_fb_controller();
    void ref_update();
    void clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg);

    Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
    Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    //Eigen::MatrixXf k_k = Eigen::MatrixXf(1,8); // declaring 1X6 Eigen matrix of datatype float
    Eigen::VectorXf igor_state = Eigen::VectorXf(6); // declaring 6x1 Eigen vector of datatype float;
    Eigen::VectorXf ref_state = Eigen::VectorXf(6);
    //Eigen::Vector3d robot_center_pos;
    Eigen::Vector3d CoM_pos;
    Eigen::Vector3d CoM_accl;
    Eigen::Vector3d zram;
    Eigen::Vector3d gravity_vec{0,0,-9.81};
    Eigen::Vector3d f;
    Eigen::Vector3d rightLegTranslation;
    Eigen::Vector3d leftLegTranslation;
    Eigen::Vector3d groundPoint;
    Eigen::Vector3d CoM_vec;
    Eigen::Vector3d CoM_line;
    Eigen::Matrix3d pitchRotEigen;

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

    // CT gains for ff_fb_controller
    // float Kp1 = -7*1.3; // Linear postion gain
    // float Kp2 = -50*0.5; // Yaw gain
    // float Kp3 = -95*0.6;//-105; // Pitch gain
    // float Kv1 = -5*0.53; // Linear velocity gain
    // float Kv2 = -10*0.3; // Yaw speed gain
    // float Kv3 = -20*0.65; // Pitch speed gain

    float Kp1 = -6.3; // Linear postion gain
    float Kp2 = -60; // Yaw gain
    float Kp3 = -95;//-105; // Pitch gain
    float Kv1 = -4; // Linear velocity gain
    float Kv2 = -10; // Yaw speed gain
    float Kv3 = -20; // Pitch speed gain
    
    Eigen::Vector3d feedbck;
    Eigen::Vector2d output_trq;

    //Eigen::MatrixXf transf_matrix = Eigen::MatrixXf(4,4);
    //tf::Vector3 CoM_tf;
    //tf::Vector3 unit_tf;
    //tf::Vector3 rot_axis{0,1,0};
    tf::Matrix3x3 pitchRotation;
    
    ros::Time sim_time;
    ros::ServiceClient client;
    std_srvs::Empty srv;

public:

    igor_knee_control(ros::NodeHandle* nodehandle); // constructor
    ~igor_knee_control(); // destructor

    float pitch_vel_y = 0;
    float yaw_vel_z = 0;
    //float freq = 0;
    geometry_msgs::Vector3 state_vec;
    geometry_msgs::Vector3 state_vec2;
    geometry_msgs::Vector3 zram_vec;
    geometry_msgs::Vector3 f_vec;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
  
    

    // Window size is 2*m+1
    const int m1 = 12;
    const int m2 = 5;
    const int m3 = 15;
    // Polynomial Order
    const int n1 = 0;
    const int n2 = 1;
    const int n3 = 3;
    // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
    // Points are defined in range [-m;m]
    const int t1 = m1;
    const int t2 = m2;
    const int t3 = m3;
    // Derivate? 0: no derivation, 1: first derivative...
    const int d = 0;
    double result;
    gram_sg::SavitzkyGolayFilterConfig sg_conf1{m1,t1,n1,d,0.002}; // filter configuration
    gram_sg::SavitzkyGolayFilterConfig sg_conf2{m2,t2,n2,1,1}; // filter configuration
    gram_sg::SavitzkyGolayFilterConfig sg_conf3{m3,t3,n3,2,1}; // filter configuration
    
    gram_sg::SavitzkyGolayFilter f1{sg_conf1};
    gram_sg::SavitzkyGolayFilter f3{sg_conf3};
    gram_sg::SavitzkyGolayFilter f4{sg_conf3};
    gram_sg::SavitzkyGolayFilter f5{sg_conf3};
    gram_sg::SavitzkyGolayFilter pitch_vel_filt{sg_conf1};
    gram_sg::SavitzkyGolayFilter yaw_vel_filt{sg_conf1};
    //gram_sg::SavitzkyGolayFilter trq_r_filt{sg_conf1};
    //gram_sg::SavitzkyGolayFilter trq_l_filt{sg_conf1};
    
    boost::circular_buffer<double> rightTrqVector {boost::circular_buffer<double>((2*m1+1),0)}; // Initialize with 0
    boost::circular_buffer<double> leftTrqVector {boost::circular_buffer<double>((2*m1+1),0)}; // Initialize with 0
    boost::circular_buffer<double> pitchVelVector {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> yawVelVector {boost::circular_buffer<double>((2*m1+1),0)};
    
    boost::circular_buffer<double> my_data1 {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> my_data3 {boost::circular_buffer<double>((2*m3+1),-0.033)};
    boost::circular_buffer<double> my_data4 {boost::circular_buffer<double>((2*m3+1),-0.033)};
    boost::circular_buffer<double> my_data5 {boost::circular_buffer<double>((2*m3+1),-0.033)};


}; //end of class