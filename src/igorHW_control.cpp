#include "igorHW_control.h"


void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
   
    CoG_Position = msg->point;

    try
    { 
       transformStamped = tfBuffer.lookupTransform("odom", "robot_center_link" , ros::Time(0));
       centerLinkTranslation = transformStamped.transform.translation;
       centerLinkRotation = transformStamped.transform.rotation;
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    tf::quaternionMsgToTF(centerLinkRotation, quat1); // geometry_msgs::Quaternion is transformed to a tf::Quaternion
    quat1.normalize(); // normalize the quaternion in case it is not normalized
    tf::Matrix3x3(quat1).getRPY(roll, pitch, yaw); // Get roll, pitch, and yaw from the quaternion

    CoG_pitch = (atan2(CoG_Position.x, CoG_Position.z) + pitch);
    CoG_pitch = floorf(CoG_pitch*10000)/10000;

    CoGVector.push_back(CoG_pitch);
    CoG_pitch = (f4.filter(CoGVector,0));

    pitchVector.push_back(CoG_pitch);
    CoG_pitch_vel = (f1.filter(pitchVector,0))/0.002; // Divide by sampling time

    if(CoG_pitch_vel > 5){
        CoG_pitch_vel = 5;
    }
    else if(CoG_pitch_vel < -5)
    {
        CoG_pitch_vel = -5;
    }

    

    

    //plot_vector.x = CoG_pitch_vel;
    //plot_vector.y = CoG_pitch;
    



    // ROS_INFO("Center link roll: %f", roll);
    // ROS_INFO("Center link pitch: %f", pitch);
    // ROS_INFO("Center link yaw: %f", yaw);
    ROS_INFO("CoG pitch: %f", CoG_pitch);
    // ROS_INFO("Center link X: %f", centerLinkTranslation.x);
    

}// End of CoG_callback


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    basePose = msg->pose.pose;
    baseTwist = msg->twist.twist;
    
    baseLinkTranslation = basePose.position;
    baseX = baseLinkTranslation.x;
    baseY = baseLinkTranslation.y;

    baseLinkRotation = basePose.orientation;
    tf::quaternionMsgToTF(baseLinkRotation, quat2); // geometry_msgs::Quaternion is transformed to a tf::Quaternion
    quat2.normalize(); // normalize the quaternion in case it is not normalized
    tf::Matrix3x3(quat2).getRPY(baseRoll, basePitch, baseYaw); // Get roll, pitch, and yaw from the quaternion

    baseLinearVelocity = baseTwist.linear;
    baseAngularVelocity = baseTwist.angular;

    basePitchVelocity = baseAngularVelocity.y;
    baseYawVelocity = baseAngularVelocity.z;
    baseXVelocity = baseLinearVelocity.x;
    baseYVelocity = baseLinearVelocity.y;

    trig_vec(0) = cos(floorf(baseYaw*1000)/1000);
    trig_vec(1) = sin(floorf(baseYaw*1000)/1000);
    pos_vec(0,0) = baseX;
    pos_vec(0,1) = baseY;
    vel_vec(0,0) = baseXVelocity;
    vel_vec(0,1) = baseYVelocity;
    igorForwardPosition = (pos_vec*trig_vec).value();
    igorForwardVel = (vel_vec*trig_vec).value();

    igorState(0) = 0*igorForwardPosition;
    igorState(3) = 0*igorForwardVel;

    igorState(1) = 0*floorf(baseYaw*10000)/10000;
    igorState(4) = 0*baseYawVelocity;

    igorState(2) = basePitch;
    igorState(5) = basePitchVelocity;

    plot_vector.x = basePitch;
    //plot_vector.y = CoG_pitch;

    //CT_controller(igorState); // Calling CT controller
    PID_controller();

} // End of odom_callback

void ref_update(){

    
    // Reference states
    refState(0) = 0.0; // Center Position 
    refState(1) = 0.0; // Yaw
    refState(2) = -0.0674; // Pitch
    refState(3) = 0.0; // Center velocity
    refState(4) = 0.0; // Yaw velocity
    refState(5) = 0.0; // Pitch velocity

    return;


} // End of ref_update

void CT_controller(Eigen::VectorXf vec) // Computed Torque controller
{
    
    ref_update();


    velocities(0) = vec(3); // Center velocity
    velocities(1) = vec(4); // Yaw velocity
    velocities(2) = vec(5); // Pitch velocity

    // Inertia matrix
    M_h(0,0)= 6.9700;
    M_h(0,1)= 0;
    M_h(0,2) = 5.9200*L*cos(vec(2));
    M_h(1,0)= 0;
    M_h(1,1)= 5.9200*pow(L,2) - pow(cos(vec(2)),2)*(5.9200*pow(L,2) + 0.0320) + 0.1449;
    M_h(1,2)= 0;
    M_h(2,0) = 5.9200*L*cos(vec(2));
    M_h(2,1)= 0;
    M_h(2,2)= 5.9200*pow(L,2) + 0.0454;

   // Coriolis and centrifugal vector 
    H_h(0) = -5.9200*L*sin(vec(2))*(pow(vec(4),2) + pow(vec(5),2));
    H_h(1) = vec(4)*(5.92000* vec(5)*sin(2*vec(2))*pow(L,2) + 5.92000*vec(3)*sin(vec(2))*L + 0.03197*vec(5)*sin(2*vec(2)));
    H_h(2) = -0.5000*pow(vec(4),2)*sin(2*vec(2))*(5.9200*pow(L,2) + 0.0320);

    // Gravity vector
    G_h(0) = 0;
    G_h(1) = 0;
    G_h(2) = -58.0752*L*sin(vec(2));

    // Position errors
    Ep(0) = vec(0)-refState(0);
    Ep(1) = vec(1)-refState(1);
    Ep(2) = vec(2)-refState(2);
    
    // Velocity errors
    Ev(0) = vec(3)-refState(3);
    Ev(1) = vec(4)-refState(4);
    Ev(2) = vec(5)-refState(5);
    

    feedbck = Kv*Ev + Kp*Ep; 
    output_trq = E_h_inv*(M_h*(feedbck)+ H_h + V_h*velocities + G_h);
    
    trq_r = 10*output_trq(1); // Right wheel torque
    trq_l = 10*output_trq(0); // Left wheel torque

    // rightTrqVector.push_back(trq_r);
    // trq_r = f2.filter(rightTrqVector,0);
    // leftTrqVector.push_back(trq_l);
    // trq_l = f3.filter(leftTrqVector,0); 
    
    plot_vector.z = trq_l;

    //ROS_INFO("Right Torque %f", trq_r);
    //ROS_INFO("Left Torque %f", trq_l);

    (*wheelGroupCommand).clear(); // Clearing the previous group commands
    (*wheelGroupCommand)[1].actuator().effort().set(-trq_r); // Effort command to Right wheel
    (*wheelGroupCommand)[0].actuator().effort().set(trq_l); // Effort command to Left wheel
    //wheel_group->sendCommand(*wheelGroupCommand); // Send commands

    //publisher.publish(plot_vector);


    return;

} // End of CT controller

void PID_controller(){

    pos_error = (-0.008)+igorState(2);
    error_d = (pos_error-last_err)/0.002;
    last_err = pos_error;
    error_i += pos_error*0.002;
    trq_r = trq_l = ((pos_error*kp)+(error_d*kd)+(error_i*ki));; 
    
    rightTrqVector.push_back(trq_r);
    trq_r = f2.filter(rightTrqVector,0);
    leftTrqVector.push_back(trq_l);
    trq_l = f3.filter(leftTrqVector,0); 
    // ROS_INFO("Pitch %f", igorState(2));
    // ROS_INFO("Trq %f", trq_l);

    (*wheelGroupCommand).clear(); // Clearing the previous group commands
    (*wheelGroupCommand)[1].actuator().effort().set(-trq_r); // Effort command to Right wheel
    (*wheelGroupCommand)[0].actuator().effort().set(trq_l); // Effort command to Left wheel
    //wheel_group->sendCommand(*wheelGroupCommand); // Send commands

    publisher.publish(plot_vector);


}// End of PID controller

void igorConfig(const ros::TimerEvent& e) // Lower body configuration
{

    ROSleftKneePos = 0.0;
    leftKneePos = -ROSleftKneePos;
    ROSrightKneePos = 0.0;
    rightKneePos = ROSrightKneePos;

    (*kneeGroupCommand).clear(); // Clearing the previous group commands
    (*kneeGroupCommand)[0].actuator().position().set(leftKneePos); // Position command to Left knee
    (*kneeGroupCommand)[1].actuator().position().set(rightKneePos); // Position command to Right knee
    
    ROSleftHipPos = 0.0;
    leftHipPos = ROSleftHipPos+M_PI/2;
    ROSrightHipPos = 0.0;
    rightHipPos = -(ROSrightHipPos+M_PI/2);

    (*hipGroupCommand).clear(); // Clearing the previous group commands
    (*hipGroupCommand)[1].actuator().position().set(rightHipPos); // Position command to Right hip
    (*hipGroupCommand)[0].actuator().position().set(leftHipPos); // Position command to Left hip
    
    hip_group->sendCommand(*hipGroupCommand);
    knee_group->sendCommand(*kneeGroupCommand);
    
} // End of igorConfig

int main(int argc, char **argv) 
{
    //Initialize ROS node
    ros::init(argc, argv, "igorHW_controller");

    ros::NodeHandle nh;
    
    
    /*********** Hebi Initialization **************/
    
    // Create a hebi Lookup Object
    hebi::Lookup lookup;
    // Wait 2 seconds for the module list to populate, and then print out its contents
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    auto entry_list = lookup.getEntryList();

    std::vector<std::string> family = {"Igor II"};
    std::vector<std::string> hip_names = {"hip1", "hip2"};
    std::vector<std::string> knee_names = {"knee1", "knee2"};
    std::vector<std::string> wheel_names = {"wheel1", "wheel2"};

    hip_group = lookup.getGroupFromNames(family, hip_names);
    knee_group = lookup.getGroupFromNames(family, knee_names);
    wheel_group = lookup.getGroupFromNames(family, wheel_names);

    if (!(hip_group && knee_group && wheel_group)) 
    {
        std::cout << std::endl
        << "Group not found! Check that the family and name of a module on the network" << std::endl
        << "matches what is given in the source file." << std::endl;
        return -1;
    }

    hip_group->setCommandLifetimeMs(100); // Set group command lifetime in ms
    knee_group->setCommandLifetimeMs(100); // Set group command lifetime in ms
    wheel_group->setCommandLifetimeMs(100); // Set group command lifetime in ms

    hipGroupCommand = new hebi::GroupCommand(hip_group->size());
    kneeGroupCommand = new hebi::GroupCommand(knee_group->size());
    wheelGroupCommand = new hebi::GroupCommand(wheel_group->size());

    // Actuator's gain upload
    // (*kneeGroupCommand).readGains("/home/fahadraza/catkin_ws/src/igor/config/HWkneeGains.xml");
    // (*kneeGroupCommand).readSafetyParameters("/home/fahadraza/catkin_ws/src/igor/config/HWkneeSafetyParam.xml");
    // bool kneeGainSuccess = knee_group->sendCommandWithAcknowledgement(*kneeGroupCommand);
    // std::cout<<"Knee Gains Uploaded:  "<< kneeGainSuccess << std::endl;

    // (*hipGroupCommand).readGains("/home/fahadraza/catkin_ws/src/igor/config/HWhipGains.xml");
    // (*hipGroupCommand).readSafetyParameters("/home/fahadraza/catkin_ws/src/igor/config/HWhipSafetyParam.xml");
    // bool hipGainSuccess = hip_group->sendCommandWithAcknowledgement(*hipGroupCommand);
    // std::cout<<"Hip Gains Uploaded:  "<< hipGainSuccess << std::endl;

    (*wheelGroupCommand).readGains("/home/fahadraza/catkin_ws/src/igor/config/HWwheelGains.xml");
    bool wheelGainSuccess = wheel_group->sendCommandWithAcknowledgement(*wheelGroupCommand);
    std::cout<<"Wheel Gains Uploaded:  "<< wheelGainSuccess << std::endl;
        
    /***********************************************/

    ros::Timer timer = nh.createTimer(ros::Duration(0.002), igorConfig); // Running at 500Hz in a separate thread
    
    
    tf2Listener = new tf2_ros::TransformListener(tfBuffer);
    publisher = nh.advertise<geometry_msgs::Vector3>( "/igor/plotVec", 5);

    // Computed-torque controller's gain
    Kp(0,0) = Kp1;
    Kp(0,1) = 0;
    Kp(0,2) = 0;
    Kp(1,0) = 0;
    Kp(1,1) = Kp2;
    Kp(1,2) = 0;
    Kp(2,0) = 0;
    Kp(2,1) = 0;
    Kp(2,2) = Kp3;

    Kv(0,0) = Kv1;
    Kv(0,1) = 0;
    Kv(0,2) = 0;
    Kv(1,0) = 0;
    Kv(1,1) = Kv2;
    Kv(1,2) = 0;
    Kv(2,0) = 0;
    Kv(2,1) = 0;
    Kv(2,2) = Kv3;

    // Viscous friction matrix
    V_h(0,0) = 19.3750;  
    V_h(0,1) = 0;
    V_h(0,2) = -1.9685;
    V_h(1,0) =  0; 
    V_h(1,1) =  1.2109;
    V_h(1,2) =  0;
    V_h(2,0) = -1.9685;
    V_h(2,1) =  0; 
    V_h(2,2) =  0.200;

    // Torque selection matrix
    E_h_inv(0,0) = 0.0502811;   
    E_h_inv(0,1) = 0.1444836;  
    E_h_inv(0,2) = -0.0051086;
    E_h_inv(1,0) = 0.0502811;  
    E_h_inv(1,1) = -0.1444836;  
    E_h_inv(1,2) = -0.0051086;



    ros::Duration(2).sleep(); // Sleep for 2 seconds
    

    CoG_sub = nh.subscribe<geometry_msgs::PointStamped>("/cog/robot",2, CoG_callback);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered",2, odom_callback, ros::TransportHints().tcpNoDelay());
    
    
        
    //ros::spin();
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads for 2 callbacks in parallel
    spinner.spin(); // spin() will not return until the node has been shutdown
  
    

    return 0;
}// end of main