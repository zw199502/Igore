#pragma once

#include <cstdio>
#include "ros/ros.h"
#include <ros/console.h>
#include <Eigen/Dense>
#include <math.h>
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>

hebi::GroupCommand* kneeGroupCommand;
hebi::GroupCommand* hipGroupCommand;
hebi::GroupCommand* wheelGroupCommand;
std::shared_ptr<hebi::Group> knee_group = NULL;
std::shared_ptr<hebi::Group> hip_group = NULL;
std::shared_ptr<hebi::Group> wheel_group = NULL;

ros::Subscriber CoG_sub;
ros::Subscriber odom_sub;
ros::Publisher  publisher;

void CT_controller(Eigen::VectorXf vec); // Function prototype, its declaration
void ref_update();


geometry_msgs::Point CoG_Position;
float CoG_pitch = 0; // Pitch angle of CoG
float CoG_pitch_vel = 0; // pitch velocity of CoG
double roll, pitch, yaw = 0.0;
double baseRoll, basePitch, baseYaw, baseX, baseY = 0.0;
float baseYawVelocity, baseXVelocity, baseYVelocity = 0;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tf2Listener;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::Vector3 centerLinkTranslation;
geometry_msgs::Quaternion centerLinkRotation;
geometry_msgs::Quaternion baseLinkRotation;
geometry_msgs::Point baseLinkTranslation;
geometry_msgs::Vector3 baseLinearVelocity;
geometry_msgs::Vector3 baseAngularVelocity;
geometry_msgs::Vector3 plot_vector;
geometry_msgs::Pose basePose;
geometry_msgs::Twist baseTwist;
tf::Quaternion quat1, quat2;

Eigen::Vector2f trig_vec;
Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
Eigen::VectorXf igorState = Eigen::VectorXf(6);
Eigen::VectorXf refState = Eigen::VectorXf(6);

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
float L = 0.513; // CoM height
float trq_r, trq_l = 0;

float igorForwardPosition = 0;
float igorForwardVel = 0;

float ROSleftKneePos = 0;
float leftKneePos = 0;
float ROSrightKneePos = 0;
float rightKneePos = 0;

float ROSleftHipPos = 0;
float leftHipPos = 0;
float ROSrightHipPos = 0;
float rightHipPos = 0;

// Window size is 2*m+1 for Savitzky Golay filter
const int m = 8;
const int n = 1;
const int t = m;

gram_sg::SavitzkyGolayFilterConfig sg_conf{m,t,n,1,1};
gram_sg::SavitzkyGolayFilter f1{sg_conf}, f2{sg_conf};
boost::circular_buffer<double> pitchVector {boost::circular_buffer<double>((2*m+1),0)}; // Initialize with 0
boost::circular_buffer<double> yawVector {boost::circular_buffer<double>((2*m+1),0)}; // Initialize with 0