#pragma once

#include <cstdio>
#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <tf2_ros/transform_broadcaster.h>

class odom_broadcaster
{
private:
    geometry_msgs::TransformStamped transform_;
    geometry_msgs::TransformStamped transform2_;
    geometry_msgs::PoseWithCovariance igor_robot_pose_;

    tf2_ros::TransformBroadcaster br;
    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber odom_sub_; // creating ROS subscriber

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

public:
    
    odom_broadcaster(ros::NodeHandle* nodehandle); // constructor
    ~odom_broadcaster(); // destructor


}; // end of class