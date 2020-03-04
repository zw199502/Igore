#pragma once
#include <cstdio>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include <ros/service.h>
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "geometry_msgs/Wrench.h"
#include <string>


class disturbance
{

private:
    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_clk; // creating ROS subscriber
    void clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg);
    bool run = false;
    ros::ServiceClient client; // ros service client to conect with gazebo service server 
    std::string igor_body_name = "base_link";
    std::string igor_reference_frame = "base_link";
    geometry_msgs::Wrench igor_wrench;
    gazebo_msgs::ApplyBodyWrench srv;

public:
    disturbance(); //constructor
    ~disturbance(); // destructor
    ros::Time my_time;
    
}; //end of class