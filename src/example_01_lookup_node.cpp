
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
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <cmath> // provides M_PI
#include <typeinfo>

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_01_lookup_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(400);

  tf::Quaternion tf_quat;
  geometry_msgs::Quaternion geom_quat;
  double roll, pitch, yaw;

  sensor_msgs::JointState igor_joint_states;
  ros::Publisher  joint_pub;
  joint_pub = n.advertise<sensor_msgs::JointState>( "/joint_states", 1 );

  std::string group_name = "my_group";
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
  
  
  std::vector<std::string> families = {"X5-1"};
  std::vector<std::string> names = {"Shoulder", "Elbow"};

  std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames(families, names);
  
  // providing "*" as the family selects all modules
  //std::shared_ptr<hebi::Group> group = lookup.getGroupFromFamily("*");

  if (!group) 
  {
    std::cout << std::endl
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  std::cout << "Group size " << group->size() << std::endl; 

  group->setCommandLifetimeMs(20);
  hebi::GroupCommand groupCommand(group->size());
  hebi::GroupFeedback feedback(group->size());
  // This effectively limits the loop below to 200Hz
  group->setFeedbackFrequencyHz(200);
  
  
  Eigen::Vector2d w;
  w(0) = (2.0 * M_PI);
  w(1) = 0*(4.0 * M_PI);
  Eigen::Vector2d w_t(0,0);
  double t = 0.0;
  double dt = 0.01; // 10 ms
  while (ros::ok()) 
  {
    w_t = w * t;
    groupCommand.setPosition(w_t.array().cos());
    groupCommand.setVelocity(0.5*w_t.array().sin());
    groupCommand[1].actuator().position().set(0); // Position command to Elbow only
    //groupCommand[0].actuator().position().set(0.78); // Position command to Shoulder only
    group->sendCommand(groupCommand);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    t += dt;
    ROS_INFO("Group Commands Sent \n");

    group->getNextFeedback(feedback);
    // Retrieve positions:
    Eigen::VectorXd positions = feedback.getPosition();
    std::cout << "Position feedback: " << std::endl << positions << std::endl;

    const auto& Shoulder_position = feedback[0].actuator().position();
    if (Shoulder_position.has()){
      std::cout << "Shoulder Position feedback: " << Shoulder_position.get() << std::endl;
    }
    else{
      std::cout << "No shoulder position feedback!" << std::endl;
    }

    const auto& Elbow_orientation = feedback[1].imu().orientation();
    const auto orient_quat = Elbow_orientation.get();
    std::cout << "orient_quat type:  " << typeid(orient_quat).name() << std::endl;
    
    geom_quat.x = orient_quat.getX();
    geom_quat.y = orient_quat.getY();
    geom_quat.z = orient_quat.getZ();
    geom_quat.w = orient_quat.getW();
    
    tf::quaternionMsgToTF(geom_quat, tf_quat);
    tf_quat.normalize(); // normalize the quaternion in case it is not normalized
    
    //the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);


    if (Elbow_orientation.has()){
      std::cout << "Elbow Roll: "<< roll  << std::endl;
      std::cout << "Elbow Pitch: "<< pitch  << std::endl;
      std::cout << "Elbow Yaw: "<< yaw  << std::endl;
    }
    else{
      std::cout << "No elbow orientation feedback!" << std::endl;
    }
    
    // Retrieve velocities:
    Eigen::VectorXd velocities = feedback.getVelocity();
    std::cout << "Velocity feedback: " << std::endl << velocities << std::endl;

    // Retrieve efforts:
    Eigen::VectorXd efforts = feedback.getEffort();
    std::cout << "Effort feedback: " << std::endl << efforts << std::endl;


    Eigen::MatrixX3d gyros = feedback.getGyro(); // Matrix of dynamic rows and 3 colums
    std::cout << "Gyro feedback: " << std::endl << gyros << std::endl;

    const auto& Elbow_gyro = feedback[1].imu().gyro(); // Matrix of dynamic rows and 3 colums
    std::cout << "Elbow Gyro_x: " << std::endl << Elbow_gyro.get().getX() << std::endl;

    /** ROS JointStates publisher **/
    igor_joint_states.header.stamp = ros::Time::now();
    igor_joint_states.name.push_back("L_hfe_joint");
    igor_joint_states.name.push_back("L_kfe_joint");
    igor_joint_states.name.push_back("L_wheel_joint");
    igor_joint_states.name.push_back("R_hfe_joint");
    igor_joint_states.name.push_back("R_kfe_joint");
    igor_joint_states.name.push_back("R_wheel_joint");
    // Joint positions rads.
    igor_joint_states.position.push_back(positions(0));
    igor_joint_states.position.push_back(positions(1));
    igor_joint_states.position.push_back(0);
    igor_joint_states.position.push_back(0);
    igor_joint_states.position.push_back(0);
    igor_joint_states.position.push_back(0);
    // Joint velocities rad/s
    igor_joint_states.velocity.push_back(velocities(0));
    igor_joint_states.velocity.push_back(velocities(1));
    igor_joint_states.velocity.push_back(0);
    igor_joint_states.velocity.push_back(0);
    igor_joint_states.velocity.push_back(0);
    igor_joint_states.velocity.push_back(0);
    // Joint efforts N.m 
    igor_joint_states.effort.push_back(0);
    igor_joint_states.effort.push_back(0);
    igor_joint_states.effort.push_back(0);
    igor_joint_states.effort.push_back(0);
    igor_joint_states.effort.push_back(0);
    igor_joint_states.effort.push_back(0);
    // Publishing on the topic
    joint_pub.publish(igor_joint_states);
    //Clearing the arrays
    igor_joint_states.name.clear();
    igor_joint_states.position.clear();
    igor_joint_states.velocity.clear();
    igor_joint_states.effort.clear();
  }
  /**##############################**/
  return 0;
}

