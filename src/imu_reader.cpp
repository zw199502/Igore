#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <vector>
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>
#include <math.h>



class imu_read
{

public:

    imu_read() //Constructor
    {
        sub = nh.subscribe<sensor_msgs::Imu>("/igor/imu/data",10,&imu_read::callback,this);
        pub = nh.advertise<geometry_msgs::Vector3>( "/igor/stateVec", 10 );
        cmd_pub1 = nh.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 10 );
        cmd_pub2 = nh.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 10 );

        k_vec(0,0)= 4.0004;
        k_vec(0,1)= 5.0604;
        k_vec(0,2)= 1.9215;
    }

    void callback(const sensor_msgs::Imu::ConstPtr &msg){
        

        igor_orient = msg->orientation;
        //igor_ang_vel = msg->angular_velocity;
        angular_velocity_y = floorf(msg->angular_velocity.y*10000)/10000; // round off data upto 4 decimal points
        //imu_header = msg->header;
        
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(igor_orient, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // the found angles saved in a geometry_msgs::Vector3
        
        rpy.x = roll;
        rpy.y = pitch;
        rpy.z = yaw;
        
        
        ang_accl = ((angular_velocity_y)-(last_vel))/dt; // Angular accel. about Y-axis in rad/sec^2
        ang_accl = floorf(ang_accl*10000)/10000; // round off data upto 4 decimal points
        last_vel = angular_velocity_y;
        state_vec.x = ang_accl;
        state_vec.y = angular_velocity_y;
        state_vec.z = pitch;

        eig_vec(0) = (ang_accl); // assigning value to state vector (Eigen Class)
        eig_vec(1) = (angular_velocity_y);
        eig_vec(2) = floorf(pitch*1000)/1000; // round off data upto 3 decimal points

        trq = (k_vec*eig_vec) ; // Eigen matrix-Vector multiplication
        std::cout<< "State Vector= " << eig_vec << std::endl;
        //std::cout<< "k-matrix= " << k_vec << std::endl;
        std::cout<< "Torq= " << trq << std::endl;
        
        trq2.data = trq.value(); // taking the scalar value of the eigen-matrx and saving into std::Float64 data slot

        this->cmd_pub(trq2);

        this->statePub(state_vec);
        
        //ROS_INFO("Time stamp: %d", imu_header.stamp.sec); // simulation time in seconds 
       // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
        //ROS_INFO("Torque: %f", trq2);
        //ROS_INFO("My angular accel y: %f", ang_accl);

    
    }// End of Callback

    void statePub (geometry_msgs::Vector3 x){

        pub.publish(x);

    }

    void cmd_pub (std_msgs::Float64 cmnd){

        cmd_pub1.publish(cmnd);
        cmd_pub2.publish(cmnd);
		

    }


private:
    ros::NodeHandle nh; // creating ROS NodeHandle
    ros::Subscriber sub; // creating ROS subscriber
    ros::Publisher  pub; // creating ROS publisher
    ros::Publisher  cmd_pub1; // creating ROS publisher
    ros::Publisher  cmd_pub2; // creating ROS publisher

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::Vector3 igor_ang_vel; // Vector3 type variable
    std_msgs::Header  imu_header; // Header type variable 
    float dt = 0.01; //sampling time of IMU (100Hz)
    float last_vel = 0.0;
    float ang_accl = 0.0;
    double roll, pitch, yaw;
    geometry_msgs::Vector3 rpy;
    geometry_msgs::Vector3 state_vec;
    float angular_velocity_y;

    //std::vector<float> y = std::vector<float>(3,0); // vector of size 3 with all entries = 0
    
    Eigen::MatrixXf trq = Eigen::MatrixXf (1,1); // declaring 1X1 Eigen matrix of datatype float
    Eigen::Vector3f eig_vec; // declaring 3X1 Eigen vector of datatype float
    Eigen::MatrixXf k_vec = Eigen::MatrixXf(1,3); // declaring 1X3 Eigen matrix of datatype float
    std_msgs::Float64 trq2;





};// End of class


int main(int argc, char **argv){


ros::init(argc, argv, "igor_imu_reader");

imu_read myNode; // creating the imu_read object


ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

return 0;

}