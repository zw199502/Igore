
#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <cmath> // provides M_PI

using namespace hebiros;


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_01_lookup_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(400);

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
  w(0) = 0*(2.0 * M_PI);
  w(1) = 0*(4.0 * M_PI);
  Eigen::Vector2d w_t(0,0);
  double t = 0.0;
  double dt = 0.01; // 10 ms
  while (ros::ok()) 
  {
    w_t = 0* w * t;
    std::cout << "w_t = " << w_t << std::endl;
    groupCommand.setPosition(w_t.array().cos());
    groupCommand.setVelocity(0.5*w_t.array().sin());
    groupCommand[1].actuator().position().set(0.78); // Position command to Elbow only
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
    // Retrieve velocities:
    Eigen::VectorXd velocities = feedback.getVelocity();
    std::cout << "Velocity feedback: " << std::endl << velocities << std::endl;

    // Retrieve efforts:
    Eigen::VectorXd efforts = feedback.getEffort();
    std::cout << "Effort feedback: " << std::endl << efforts << std::endl;


    Eigen::MatrixX3d gyros = feedback.getGyro(); // Matrix of dynamic rows and 3 colums
    std::cout << "Gyro feedback: " << std::endl << gyros << std::endl;

  }
  /**##############################**/
  //Create a client which uses the service to see the entry list of modules
  ros::ServiceClient entry_list_client = n.serviceClient<EntryListSrv>("/hebiros/entry_list");

  ROS_INFO("entry_list_client created");

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("/hebiros/add_group_from_names");

  ROS_INFO("Group created");

  //Create a client which uses the service to find the size of a group
  ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");

  ROS_INFO("Size of Group created");

  EntryListSrv entry_list_srv;
  AddGroupFromNamesSrv add_group_srv;
  SizeSrv size_srv;

  //Call the entry_list service, displaying each module on the network
  //entry_list_srv.response.entry_list will now be populated with those modules
  entry_list_client.call(entry_list_srv);
  ROS_INFO("entry_list service called");

  //Construct a group using 2 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"Shoulder", "Elbow"};
  add_group_srv.request.families = {"X5-1"};
  ROS_INFO("2 module group constructed");
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
 // while(!add_group_client.call(add_group_srv)) {
 //   ROS_INFO("In while loop");
 // }

  //Call the size service for the newly created group
  size_client.call(size_srv);
  ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);

  //Spin
  //The group will exist until the node is shutdown
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

