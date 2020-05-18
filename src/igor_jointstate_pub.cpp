#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>


int main(int argc, char **argv) 
{

    //Initialize ROS node
    ros::init(argc, argv, "igor_jointState_publisher");
    ros::NodeHandle nh_;
    
    double wheel_rad = 0.1016;
    double lengthBetweenTwoWheels = 0.43;
    double x = 0;
    double y = 0;
    double th = 0;
    double delta_x = 0;
    double delta_y = 0;
    double delta_th = 0;
    double roll,pitch,yaw;

    sensor_msgs::Imu Lhip_imu;
    sensor_msgs::Imu Rhip_imu;
    sensor_msgs::Imu Rwheel_imu;
    sensor_msgs::Imu Lwheel_imu;
    nav_msgs::Odometry wheel_odom;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf::Quaternion tf_quat;
    tf::Quaternion rot_quat;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::Quaternion Lhip_quat;
    geometry_msgs::Vector3 Lhip_angular_vel;
    geometry_msgs::Vector3 Lhip_lin_accel;
    geometry_msgs::Quaternion Rhip_quat;
    geometry_msgs::Quaternion Rwheel_quat;
    geometry_msgs::Quaternion Lwheel_quat;
    geometry_msgs::Vector3 Rhip_angular_vel;
    geometry_msgs::Vector3 Rhip_lin_accel;
    geometry_msgs::Vector3 Rwheel_angular_vel;
    geometry_msgs::Vector3 Lwheel_angular_vel;
    geometry_msgs::Vector3 Rwheel_lin_accel;
    geometry_msgs::Vector3 Lwheel_lin_accel;
    geometry_msgs::TransformStamped odom_trans;
    ros::Time last_time, current_time;
    sensor_msgs::JointState igor_joint_states; // Joint states
    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("/HWjoint_states", 10); // Hardware Joint state publisher
    ros::Publisher odom_pub = nh_.advertise<nav_msgs::Odometry>("/wheel_odom", 10); // odometry publisher
    ros::Publisher Lhip_imu_pub = nh_.advertise<sensor_msgs::Imu>("/Lhip_imu/data", 10); // Left hip imu publisher
    ros::Publisher Rhip_imu_pub = nh_.advertise<sensor_msgs::Imu>("/Rhip_imu/data", 10); // Right hip imu publisher
    ros::Publisher Rwheel_imu_pub = nh_.advertise<sensor_msgs::Imu>("/Rwheel_imu/data", 10); // Right wheel imu publisher
    ros::Publisher Lwheel_imu_pub = nh_.advertise<sensor_msgs::Imu>("/Lwheel_imu/data", 10); // Left wheel imu publisher

    /** IMU covariance **/
    Lwheel_imu.orientation_covariance[0] = pow(0.005, 2);
    Lwheel_imu.orientation_covariance[4] = pow(0.005, 2);
    Lwheel_imu.orientation_covariance[8] = pow(0.005, 2);
    Lwheel_imu.angular_velocity_covariance[0] = pow(0.001, 2);
    Lwheel_imu.angular_velocity_covariance[4] = pow(0.001, 2);
    Lwheel_imu.angular_velocity_covariance[8] = pow(0.001, 2);
    Lwheel_imu.linear_acceleration_covariance[0] = pow(0.015, 2);
    Lwheel_imu.linear_acceleration_covariance[4] = pow(0.015, 2);
    Lwheel_imu.linear_acceleration_covariance[8] = pow(0.015, 2);

    Rwheel_imu.orientation_covariance[0] = pow(0.005, 2);
    Rwheel_imu.orientation_covariance[4] = pow(0.005, 2);
    Rwheel_imu.orientation_covariance[8] = pow(0.005, 2);
    Rwheel_imu.angular_velocity_covariance[0] = pow(0.001, 2);
    Rwheel_imu.angular_velocity_covariance[4] = pow(0.001, 2);
    Rwheel_imu.angular_velocity_covariance[8] = pow(0.001, 2);
    Rwheel_imu.linear_acceleration_covariance[0] = pow(0.015, 2);
    Rwheel_imu.linear_acceleration_covariance[4] = pow(0.015, 2);
    Rwheel_imu.linear_acceleration_covariance[8] = pow(0.015, 2);

    Lhip_imu.orientation_covariance[0] = pow(0.005, 2);
    Lhip_imu.orientation_covariance[4] = pow(0.005, 2);
    Lhip_imu.orientation_covariance[8] = pow(0.005, 2);
    Lhip_imu.angular_velocity_covariance[0] = pow(0.001, 2);
    Lhip_imu.angular_velocity_covariance[4] = pow(0.001, 2);
    Lhip_imu.angular_velocity_covariance[8] = pow(0.001, 2);
    Lhip_imu.linear_acceleration_covariance[0] = pow(0.015, 2);
    Lhip_imu.linear_acceleration_covariance[4] = pow(0.015, 2);
    Lhip_imu.linear_acceleration_covariance[8] = pow(0.015, 2);

    Rhip_imu.orientation_covariance[0] = pow(0.005, 2);
    Rhip_imu.orientation_covariance[4] = pow(0.005, 2);
    Rhip_imu.orientation_covariance[8] = pow(0.005, 2);
    Rhip_imu.angular_velocity_covariance[0] = pow(0.001, 2);
    Rhip_imu.angular_velocity_covariance[4] = pow(0.001, 2);
    Rhip_imu.angular_velocity_covariance[8] = pow(0.001, 2);
    Rhip_imu.linear_acceleration_covariance[0] = pow(0.015, 2);
    Rhip_imu.linear_acceleration_covariance[4] = pow(0.015, 2);
    Rhip_imu.linear_acceleration_covariance[8] = pow(0.015, 2);
    /*****************************************************************///

    // Create a hebi Lookup Object
    hebi::Lookup lookup;
    // Wait 2 seconds for the module list to populate, and then print out its contents
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    std::cout << std::endl;
    auto entry_list = lookup.getEntryList();

    for (auto entry : *entry_list)
    {
        std::cout
        << "Name: " << entry.name_ << std::endl
        << "Family: " << entry.family_ << std::endl << std::endl;
    }

    std::vector<std::string> family = {"Igor II"};
    std::vector<std::string> hip_names = {"hip1", "hip2"};
    std::vector<std::string> knee_names = {"knee1", "knee2"};
    std::vector<std::string> wheel_names = {"wheel1", "wheel2"};

    std::shared_ptr<hebi::Group> hip_group = lookup.getGroupFromNames(family, hip_names);
    std::shared_ptr<hebi::Group> knee_group = lookup.getGroupFromNames(family, knee_names);
    std::shared_ptr<hebi::Group> wheel_group = lookup.getGroupFromNames(family, wheel_names);

    hip_group->setFeedbackFrequencyHz(500);
    knee_group->setFeedbackFrequencyHz(500);
    wheel_group->setFeedbackFrequencyHz(500);
    
    // providing "*" as the family selects all modules
    //std::shared_ptr<hebi::Group> group = lookup.getGroupFromFamily("*");

    if (!(hip_group && knee_group && wheel_group)) 
    {
        std::cout << std::endl
        << "Group not found! Check that the family and name of a module on the network" << std::endl
        << "matches what is given in the source file." << std::endl;
        return -1;
    }
    std::cout << "Hip group size " << hip_group->size() << std::endl;
    std::cout << "Knee group size " << knee_group->size() << std::endl;
    std::cout << "Wheel group size " << wheel_group->size() << std::endl;

    hebi::GroupFeedback hip_feedback(hip_group->size());
    hebi::GroupFeedback knee_feedback(knee_group->size());
    hebi::GroupFeedback wheel_feedback(wheel_group->size());

    while (ros::ok())
    {
        current_time = ros::Time::now();

        hip_group->getNextFeedback(hip_feedback);
        knee_group->getNextFeedback(knee_feedback);
        wheel_group->getNextFeedback(wheel_feedback);

        Eigen::VectorXd hip_positions = hip_feedback.getPosition();
        Eigen::VectorXd knee_positions = knee_feedback.getPosition();
        Eigen::VectorXd wheel_positions = wheel_feedback.getPosition();

        Eigen::VectorXd hip_velocities = hip_feedback.getVelocity();
        Eigen::VectorXd knee_velocities = knee_feedback.getVelocity();
        Eigen::VectorXd wheel_velocities = wheel_feedback.getVelocity();

        Eigen::VectorXd hip_efforts = hip_feedback.getEffort();
        Eigen::VectorXd knee_efforts = knee_feedback.getEffort();
        Eigen::VectorXd wheel_efforts = wheel_feedback.getEffort();

        /** ROS JointStates publisher **/
        igor_joint_states.header.stamp = current_time;
        igor_joint_states.name.push_back("L_hfe_joint");
        igor_joint_states.name.push_back("L_kfe_joint");
        igor_joint_states.name.push_back("L_wheel_joint");
        igor_joint_states.name.push_back("R_hfe_joint");
        igor_joint_states.name.push_back("R_kfe_joint");
        igor_joint_states.name.push_back("R_wheel_joint");
        // Joint positions rads.
        igor_joint_states.position.push_back(hip_positions(0)-M_PI/2); // Left hip
        igor_joint_states.position.push_back(-knee_positions(0)); // Left knee
        igor_joint_states.position.push_back(wheel_positions(0)); // Left wheel
        igor_joint_states.position.push_back(-hip_positions(1)-M_PI/2); // Right hip
        igor_joint_states.position.push_back(knee_positions(1)); // Right knee
        igor_joint_states.position.push_back(-wheel_positions(1)); // Right wheel 
       
        // Joint velocities rad/s
        igor_joint_states.velocity.push_back(hip_velocities(0));
        igor_joint_states.velocity.push_back(-knee_velocities(0));
        igor_joint_states.velocity.push_back(wheel_velocities(0));
        igor_joint_states.velocity.push_back(-hip_velocities(1));
        igor_joint_states.velocity.push_back(knee_velocities(1));
        igor_joint_states.velocity.push_back(-wheel_velocities(1));
      
        // Joint efforts N.m 
        igor_joint_states.effort.push_back(hip_efforts(0));
        igor_joint_states.effort.push_back(knee_efforts(0));
        igor_joint_states.effort.push_back(-wheel_efforts(0)); // Left Wheel
        igor_joint_states.effort.push_back(hip_efforts(1));
        igor_joint_states.effort.push_back(knee_efforts(1));
        igor_joint_states.effort.push_back(wheel_efforts(1)); // Right Wheel
       

        /************** Wheel odometry**************/
        double v_left = wheel_velocities(0) * wheel_rad; //Linear velocity left wheel
        double v_right = -wheel_velocities(1) * wheel_rad; //Linear velocity right wheel
        
        double vx = ((v_right + v_left) / 2); // forward velocity
        double vy = 0; // sideways velocity
        double vth = ((v_right - v_left)/lengthBetweenTwoWheels); // yaw velocity
        
        double dt = (current_time - last_time).toSec(); // Time delta
        if(dt>=1){
            dt = 0.002; // 500 hz
        }
        
        delta_x = (vx * cos(th)) * dt;
        delta_y = (vx * sin(th)) * dt;
        delta_th = vth * dt;

        x += delta_x; // X position
        y += delta_y; // Y position
        th += delta_th; // yaw angle
        //ROS_INFO("X position: %f", x);
        //ROS_INFO("Yaw angle: %f", th);
        odom_quat = tf::createQuaternionMsgFromYaw(th); // Creating quaternion
       


        // nav odom
        wheel_odom.header.stamp = current_time;
        wheel_odom.header.frame_id = "odom";
        wheel_odom.child_frame_id = "base_link";

        //set the position
        wheel_odom.pose.pose.position.x = x;
        wheel_odom.pose.pose.position.y = y;
        wheel_odom.pose.pose.position.z = 0.0;
        wheel_odom.pose.pose.orientation = odom_quat;
        wheel_odom.pose.covariance[0] = 0.01125; // X
        wheel_odom.pose.covariance[7] = 0.01125; // Y
        wheel_odom.pose.covariance[14] = 0.00; // Z
        wheel_odom.pose.covariance[21] = 0.00; // Roll
        wheel_odom.pose.covariance[28] = 0.00; // Pitch
        wheel_odom.pose.covariance[35] = 0.001; // Yaw

        //set the velocity
        wheel_odom.twist.twist.linear.x = vx;
        wheel_odom.twist.twist.linear.y = vy;
        wheel_odom.twist.twist.angular.z = vth;
        wheel_odom.twist.covariance[0] = 0.001; // X velocity
        wheel_odom.twist.covariance[7] = 0.001; // Y velocity
        wheel_odom.twist.covariance[14] = 0.00; // Z velocity
        wheel_odom.twist.covariance[21] = 0.00; // Roll velocity
        wheel_odom.twist.covariance[28] = 0.00; // Pitch velocity
        wheel_odom.twist.covariance[35] = 0.001; // Yaw velocity

        /***********************************/
        
        /*******IMU Readings**************/
        
        // Left hip orientation
        const auto& Lhip_orientation = hip_feedback[0].imu().orientation(); 
        Lhip_quat.x = Lhip_orientation.get().getX();
        Lhip_quat.y = Lhip_orientation.get().getY();
        Lhip_quat.z = Lhip_orientation.get().getZ();
        Lhip_quat.w = Lhip_orientation.get().getW();
        
        
        // Right hip orientation
        const auto& Rhip_orientation = hip_feedback[1].imu().orientation(); // Right hip orientation
        Rhip_quat.x = Rhip_orientation.get().getX();
        Rhip_quat.y = Rhip_orientation.get().getY();
        Rhip_quat.z = Rhip_orientation.get().getZ();
        Rhip_quat.w = Rhip_orientation.get().getW();

        /** Useless part **/
        // tf::quaternionMsgToTF(Rhip_quat, tf_quat);
        
        // rot_quat.setRPY(0,0,0);
        // tf_quat = tf_quat*rot_quat;
        // tf_quat.normalize();
        // rot_quat.setRPY(0,0,0);
        // tf_quat = tf_quat*rot_quat;
        // tf_quat.normalize(); // normalize the quaternion in case it is not normalized
        //the tf::Quaternion has a method to acess roll pitch and yaw
        // tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        // ROS_INFO("Roll: %f", roll);
        // ROS_INFO("Pitch: %f", pitch);
        // ROS_INFO("Yaw: %f", yaw);
        // tf_quat.setRPY(0, pitch, yaw); //Adapting HW values to ROS coordinate frames
        //tf_quat.setEulerZYX(yaw,roll,pitch);
        //tf::quaternionTFToMsg(tf_quat, Rhip_quat); // Converting tf quaternion in to geometry_msgs quaternion
        /*************************/

        // Right wheel orientation
        const auto& Rwheel_orientation = wheel_feedback[1].imu().orientation(); // Right wheel orientation
        Rwheel_quat.x = Rwheel_orientation.get().getX();
        Rwheel_quat.y = Rwheel_orientation.get().getY();
        Rwheel_quat.z = Rwheel_orientation.get().getZ();
        Rwheel_quat.w = Rwheel_orientation.get().getW();
        

        // Left wheel orientation
        const auto& Lwheel_orientation = wheel_feedback[0].imu().orientation(); // Left wheel orientation
        Lwheel_quat.x = Lwheel_orientation.get().getX();
        Lwheel_quat.y = Lwheel_orientation.get().getY();
        Lwheel_quat.z = Lwheel_orientation.get().getZ();
        Lwheel_quat.w = Lwheel_orientation.get().getW();

        // creating tf transform
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = Rhip_quat;
        // //send the transform
        //odom_broadcaster.sendTransform(odom_trans);
        
        // Left hip rotational velocities
        const auto& Lhip_gyro = hip_feedback[0].imu().gyro(); 
        Lhip_angular_vel.x = Lhip_gyro.get().getX();
        Lhip_angular_vel.y = Lhip_gyro.get().getY();
        Lhip_angular_vel.z = Lhip_gyro.get().getZ();

        // Right hip rotational velocities
        const auto& Rhip_gyro = hip_feedback[1].imu().gyro(); 
        Rhip_angular_vel.x = Rhip_gyro.get().getX();
        Rhip_angular_vel.y = Rhip_gyro.get().getY();
        Rhip_angular_vel.z = Rhip_gyro.get().getZ();

        // Right wheel rotational velocities
        const auto& Rwheel_gyro = wheel_feedback[1].imu().gyro(); 
        Rwheel_angular_vel.x = Rwheel_gyro.get().getX();
        Rwheel_angular_vel.y = Rwheel_gyro.get().getY();
        Rwheel_angular_vel.z = Rwheel_gyro.get().getZ();

        // Left wheel rotational velocities
        const auto& Lwheel_gyro = wheel_feedback[0].imu().gyro(); 
        Lwheel_angular_vel.x = Lwheel_gyro.get().getX();
        Lwheel_angular_vel.y = Lwheel_gyro.get().getY();
        Lwheel_angular_vel.z = Lwheel_gyro.get().getZ();

 
        // Left hip Linear accelerations
        const auto& Lhip_accel = hip_feedback[0].imu().accelerometer(); 
        Lhip_lin_accel.x = Lhip_accel.get().getX();
        Lhip_lin_accel.y = Lhip_accel.get().getY();
        Lhip_lin_accel.z = Lhip_accel.get().getZ();

        // Right hip Linear accelerations
        const auto& Rhip_accel = hip_feedback[1].imu().accelerometer(); 
        Rhip_lin_accel.x = Rhip_accel.get().getX();
        Rhip_lin_accel.y = Rhip_accel.get().getY();
        Rhip_lin_accel.z = Rhip_accel.get().getZ();

        // Right wheel Linear accelerations
        const auto& Rwheel_accel = wheel_feedback[1].imu().accelerometer(); 
        Rwheel_lin_accel.x = Rwheel_accel.get().getX();
        Rwheel_lin_accel.y = Rwheel_accel.get().getY();
        Rwheel_lin_accel.z = Rwheel_accel.get().getZ();

        // Left wheel Linear accelerations
        const auto& Lwheel_accel = wheel_feedback[0].imu().accelerometer(); 
        Lwheel_lin_accel.x = Lwheel_accel.get().getX();
        Lwheel_lin_accel.y = Lwheel_accel.get().getY();
        Lwheel_lin_accel.z = Lwheel_accel.get().getZ();

        // Lhip IMU
        Lhip_imu.header.stamp = current_time;
        Lhip_imu.header.frame_id = "LhipActuator_imu_link";
        Lhip_imu.orientation = Lhip_quat;
        Lhip_imu.angular_velocity = Lhip_angular_vel;
        Lhip_imu.linear_acceleration = Lhip_lin_accel;

        // Rhip IMU
        Rhip_imu.header.stamp = current_time;
        Rhip_imu.header.frame_id = "RhipActuator_imu_link";
        Rhip_imu.orientation = Rhip_quat;
        Rhip_imu.angular_velocity = Rhip_angular_vel;
        Rhip_imu.linear_acceleration = Rhip_lin_accel;

        // Rwheel IMU
        Rwheel_imu.header.stamp = current_time;
        Rwheel_imu.header.frame_id = "RwheelActuator_imu_link";
        Rwheel_imu.orientation = Rwheel_quat;
        Rwheel_imu.angular_velocity = Rwheel_angular_vel;
        Rwheel_imu.linear_acceleration = Rwheel_lin_accel;

        // Lwheel IMU
        Lwheel_imu.header.stamp = current_time;
        Lwheel_imu.header.frame_id = "LwheelActuator_imu_link";
        Lwheel_imu.orientation = Lwheel_quat;
        Lwheel_imu.angular_velocity = Lwheel_angular_vel;
        Lwheel_imu.linear_acceleration = Lwheel_lin_accel;
        

        // Publishing on the topics
        odom_pub.publish(wheel_odom);
        Rwheel_imu_pub.publish(Rwheel_imu);
        Lwheel_imu_pub.publish(Lwheel_imu);
        Lhip_imu_pub.publish(Lhip_imu);
        Rhip_imu_pub.publish(Rhip_imu);
        joint_pub.publish(igor_joint_states);
        //Clearing the arrays
        igor_joint_states.name.clear();
        igor_joint_states.position.clear();
        igor_joint_states.velocity.clear();
        igor_joint_states.effort.clear();

        last_time = current_time;
    }// end of while loop     


    return 0;
}// end of main