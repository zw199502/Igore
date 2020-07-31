#include "igorHW_control.h"


void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
   
    CoG_Position = msg->point;

    CoM_vec << CoG_Position.x, CoG_Position.y, CoG_Position.z;
    pitchRotation.setRPY(0,basePitch,0); // Setting Pitch rotation matrix
    tf::matrixTFToEigen(pitchRotation, pitchRotEigen); // Converting tf matrix to Eigen matrix

    try
    { 
        leftLegTransformStamped = leftLegTfBuffer.lookupTransform("base_link", "L_wheelActuator" , ros::Time(0));
        rightLegTransformStamped = rightLegTfBuffer.lookupTransform("base_link", "R_wheelActuator" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    leftLegTranslation << leftLegTransformStamped.transform.translation.x, leftLegTransformStamped.transform.translation.y, leftLegTransformStamped.transform.translation.z;
    rightLegTranslation << rightLegTransformStamped.transform.translation.x, rightLegTransformStamped.transform.translation.y, rightLegTransformStamped.transform.translation.z;
    // Find the mean of the two legs' translation vectors in base_link frame
    groundPoint = 0.5*(leftLegTranslation+rightLegTranslation);
    //Get the vector starting from the "ground point" and ending at the position of the current center of mass
    CoM_line = CoM_vec - groundPoint;
    // Rotate it according to the current pitch angle of Igor
    CoM_line = pitchRotEigen * CoM_line;
    CoM_height =  CoM_line.norm();
    // Lean/Pitch angle of CoM from the wheel base 
    leanAngle = atan2(CoM_line.x(), CoM_line.z());

    CoGVector.push_back(leanAngle);
    CoG_pitch = (f4.filter(CoGVector,0));

    pitchVector.push_back(CoG_pitch);
    CoG_pitch_vel = (f1.filter(pitchVector,0))/0.002; // Divide by sampling time

    if(CoG_pitch_vel > 10){
        CoG_pitch_vel = 10;
    }
    else if(CoG_pitch_vel < -10)
    {
        CoG_pitch_vel = -10;
    }

    ROS_INFO("CoM Height %f",CoM_height);
    

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

    igorState(0) = igorForwardPosition;
    igorState(3) = igorForwardVel;

    igorState(1) = floorf(baseYaw*10000)/10000;
    igorState(4) = baseYawVelocity;

    igorState(2) = CoG_pitch;
    igorState(5) = basePitchVelocity;

    // Publishing states for ploting
    plot_vector.data[0] = igorState(0); // Forward position
    //plot_vector.data[1] = igorState(1); // Yaw
    plot_vector.data[2] = igorState(2); // Pitch
    //plot_vector.data[3] = igorState(3); // Forward Velocity
    plot_vector.data[4] = igorState(4); // Yaw Velocity
    //plot_vector.data[5] = igorState(5); // Pitch Velocity
    plot_vector.data[8] = baseY;
    ROS_INFO("Pitch angle %f",igorState(2));

    //CT_controller(igorState); // Calling CT controller
    //LQR_controller(igorState); // Calling LQR controller
    ff_fb_controller();

} // End of odom_callback

void ref_update(){

    ROS_INFO("In ref update");
    wall_duration = ros::WallTime::now() - wall_begin;
    ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
    // Reference states
    if (wall_duration.sec>=10){
        //plot_vector.data[1] = refState(0) = 0.5*(sin(0.7*ros::Time::now().toSec())); // Center Position
        plot_vector.data[1] = refState(0) = 0.5; // Center Position  
        plot_vector.data[3] = refState(1) = 0*0.785398*(cos(0.3*ros::Time::now().toSec())); // Yaw
        plot_vector.data[5] = refState(2) = -0.004*0; // Pitch
        refState(3) = 0.0; // Center velocity
        refState(4) = 0.0; // Yaw velocity
        refState(5) = 0.0; // Pitch velocity
    }
    else{
        plot_vector.data[1] = refState(0) = 0.0;
        plot_vector.data[3] = refState(1) = 0.0;
        plot_vector.data[5] = refState(2) = 0.0; // Pitch
        refState(3) = 0.0; // Center velocity
        refState(4) = 0.0; // Yaw velocity
        refState(5) = 0.0; // Pitch velocity

    }
    return;

} // End of ref_update

void CT_controller(Eigen::VectorXf vec) // Computed Torque controller
{
    
    ref_update();

    ROS_INFO("In CT controller");
    ROS_INFO("Yaw Reference: %f", refState(1));
    ROS_INFO("Yaw error %f", (refState(1)-vec(1)));
    
    L = CoM_height;

    velocities(0) = vec(3); // Center velocity
    velocities(1) = vec(4); // Yaw velocity
    velocities(2) = vec(5); // Pitch velocity

    // Inertia matrix
    M_h(0,0)= 8.55;
    M_h(0,1)= 0;
    M_h(0,2) = 7.5*L*cos(vec(2));
    M_h(1,0)= 0;
    M_h(1,1)= 7.5*pow(L,2) - pow(cos(vec(2)),2)*(7.5*pow(L,2) + 0.0246) + 0.1382;
    M_h(1,2)= 0;
    M_h(2,0) = 7.5*L*cos(vec(2));
    M_h(2,1)= 0;
    M_h(2,2)= 7.5*pow(L,2) + 0.0347;

   // Coriolis and centrifugal vector 
    H_h(0) = -7.5*L*sin(vec(2))*(pow(vec(4),2) + pow(vec(5),2));
    H_h(1) = 6.0000e-04*vec(4)*(12500*vec(5)*sin(2*vec(2))*pow(L,2) + 12500*vec(3)*sin(vec(2))*L + 41*vec(5)*sin(2*vec(2)));
    H_h(2) = -0.5000*pow(vec(4),2)*sin(2*vec(2))*(7.5000*pow(L,2) + 0.0246);

    // Gravity vector
    G_h(0) = 0;
    G_h(1) = 0;
    G_h(2) = -73.5750*L*sin(vec(2));

   // Position errors
    Ep(0) = refState(0)-vec(0);
    Ep(1) = refState(1)-vec(1);
    Ep(2) = refState(2)-vec(2);
    
    // Velocity errors
    Ev(0) = refState(3)-vec(3);
    Ev(1) = refState(4)-vec(4);
    Ev(2) = refState(5)-vec(5);
    

    feedbck = Kv*Ev + Kp*Ep; 
    output_trq = E_h_inv*(M_h*(feedbck)+ H_h + V_h*velocities + G_h);
    
    CT_trq_r = output_trq(1); // Right wheel torque
    CT_trq_l = output_trq(0); // Left wheel torque

    
    

    ROS_INFO("Right Torque %f", CT_trq_r);
    ROS_INFO("Left Torque %f", CT_trq_l);

    (*wheelGroupCommand).clear(); // Clearing the previous group commands
    (*wheelGroupCommand)[1].actuator().effort().set(-CT_trq_r); // Effort command to Right wheel
    (*wheelGroupCommand)[0].actuator().effort().set(CT_trq_l); // Effort command to Left wheel
    // wheel_group->sendCommand(*wheelGroupCommand); // Send commands

    // plot_vector.data[6] = CT_trq_l;
    // plot_vector.data[7] = CT_trq_r;
    // array_publisher.publish(plot_vector);


    return;

} // End of CT controller



void LQR_controller(Eigen::VectorXf vec)
{

    ref_update();
    ROS_INFO("In LQR controller");
    ROS_INFO("Yaw Reference: %f", refState(1));

    
    lqr_trq_r =  (k_r*(refState-vec)).value(); // taking the scalar value of the eigen-matrx
        
    lqr_trq_l =  (k_l*(refState-vec)).value();

    ROS_INFO("Right LQR Torque %f", lqr_trq_r);
    ROS_INFO("Left LQR Torque %f", lqr_trq_l);
    ROS_INFO("Yaw error %f", (refState(1)-vec(1)));

    (*wheelGroupCommand).clear(); // Clearing the previous group commands
    (*wheelGroupCommand)[1].actuator().effort().set(-lqr_trq_r); // Effort command to Right wheel
    (*wheelGroupCommand)[0].actuator().effort().set(lqr_trq_l); // Effort command to Left wheel
    // wheel_group->sendCommand(*wheelGroupCommand); // Send commands

    // plot_vector.data[6] = lqr_trq_l; // left wheel torque
    // plot_vector.data[7] = lqr_trq_r; // right wheel torque
    
    // array_publisher.publish(plot_vector);

    return;

}// End of LQR controller


void ff_fb_controller() // feedforward+feedback controller
{

    ROS_INFO("In ff_fb_controller");
    ROS_INFO("Yaw Reference: %f", refState(1));

    CT_controller(igorState); // Calling CT controller
    LQR_controller(igorState); // Calling LQR controller
    trq_l = lqr_trq_l+CT_trq_l;
    trq_r = lqr_trq_r+CT_trq_r;

    (*wheelGroupCommand).clear(); // Clearing the previous group commands
    (*wheelGroupCommand)[1].actuator().effort().set(-trq_r); // Effort command to Right wheel
    (*wheelGroupCommand)[0].actuator().effort().set(trq_l); // Effort command to Left wheel
    wheel_group->sendCommand(*wheelGroupCommand); // Send commands

    plot_vector.data[6] = trq_l; // left wheel torque
    plot_vector.data[7] = trq_r; // right wheel torque
    array_publisher.publish(plot_vector);

    return;

}// End of ff_fb_controller

void igorConfig(const ros::TimerEvent& e) // Lower body configuration
{

    ROSleftKneePos = -0.65;
    leftKneePos = -ROSleftKneePos;
    ROSrightKneePos = -0.65;
    rightKneePos = ROSrightKneePos;

    (*kneeGroupCommand).clear(); // Clearing the previous group commands
    (*kneeGroupCommand)[0].actuator().position().set(leftKneePos); // Position command to Left knee
    (*kneeGroupCommand)[1].actuator().position().set(rightKneePos); // Position command to Right knee
    
    ROSleftHipPos = (1.9-M_PI/2);
    leftHipPos = ROSleftHipPos+M_PI/2;
    ROSrightHipPos = (1.9-M_PI/2);
    rightHipPos = -(ROSrightHipPos+M_PI/2);

    (*hipGroupCommand).clear(); // Clearing the previous group commands
    (*hipGroupCommand)[1].actuator().position().set(rightHipPos); // Position command to Right hip
    (*hipGroupCommand)[0].actuator().position().set(leftHipPos); // Position command to Left hip
    (*hipGroupCommand)[1].actuator().velocity().set(0.0); // Velocity command to Right hip
    (*hipGroupCommand)[0].actuator().velocity().set(0.0); // Velocity command to Left hip
    
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
    (*kneeGroupCommand).readSafetyParameters("/home/fahadraza/catkin_ws/src/igor/config/HWkneeSafetyParam.xml");
    bool kneeGainSuccess = knee_group->sendCommandWithAcknowledgement(*kneeGroupCommand);
    std::cout<<"Knee Gains Uploaded:  "<< kneeGainSuccess << std::endl;

    // (*hipGroupCommand).readGains("/home/fahadraza/catkin_ws/src/igor/config/HWhipGains.xml");
    (*hipGroupCommand).readSafetyParameters("/home/fahadraza/catkin_ws/src/igor/config/HWhipSafetyParam.xml");
    bool hipGainSuccess = hip_group->sendCommandWithAcknowledgement(*hipGroupCommand);
    std::cout<<"Hip Gains Uploaded:  "<< hipGainSuccess << std::endl;

    (*wheelGroupCommand).readGains("/home/fahadraza/catkin_ws/src/igor/config/HWwheelGains.xml");
    (*wheelGroupCommand).readSafetyParameters("/home/fahadraza/catkin_ws/src/igor/config/HWwheelSafetyParam.xml");
    bool wheelGainSuccess = wheel_group->sendCommandWithAcknowledgement(*wheelGroupCommand);
    std::cout<<"Wheel Gains Uploaded:  "<< wheelGainSuccess << std::endl;
        
    /***********************************************/

    ros::Timer timer = nh.createTimer(ros::Duration(0.002), igorConfig); // Running at 500Hz in a separate thread
    
    
    leftLegTfListener = new tf2_ros::TransformListener(leftLegTfBuffer);
    rightLegTfListener = new tf2_ros::TransformListener(rightLegTfBuffer);
    
    array_publisher = nh.advertise<std_msgs::Float32MultiArray>( "/igor/plotVec", 5);
    plot_vector.data.resize(9);

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
    V_h(0,0) = 48.4376;  
    V_h(0,1) = 0;
    V_h(0,2) = -4.9213;
    V_h(1,0) =  0; 
    V_h(1,1) =  2.2390;
    V_h(1,2) =  0;
    V_h(2,0) = -4.9213;
    V_h(2,1) =  0; 
    V_h(2,2) =  0.5000;
    
    // V_h(0,0) = 45.5313;  
    // V_h(0,1) = -0.6248;
    // V_h(0,2) = -4.6260;
    // V_h(1,0) = -0.6248; 
    // V_h(1,1) =  2.1047;
    // V_h(1,2) =  0.0635;
    // V_h(2,0) = -4.6260;
    // V_h(2,1) =  0.0635; 
    // V_h(2,2) =  0.4700;

    // Torque selection matrix
    E_h_inv(0,0) = 0.0503;   
    E_h_inv(0,1) = 0.1605;  
    E_h_inv(0,2) = -0.0051;
    E_h_inv(1,0) = 0.0503;  
    E_h_inv(1,1) = -0.1605;  
    E_h_inv(1,2) = -0.0051;


    // LQR gains for ff_fb_controller
    k_r(0,0)= k_l(0,0) = 1*(-0.7071); // Forward position gain -ve
    k_r(0,1)= 1*(0.7071); // Yaw gain +ve
    k_r(0,2)= k_l(0,2) = 1*(-16.2331); // Pitch gain -ve
    k_r(0,3)= k_l(0,3) = 0.65*(-4.8849); // Forward speed gain -ve
    k_r(0,4)= 0.5*(0.4032); // Yaw speed gain +ve
    k_r(0,5)= k_l(0,5)= 1.2*(-3.1893); // Pitch speed gain -ve
    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);

    // LQR gains
    // k_r(0,0)= k_l(0,0) = 4*(-0.7071); // Forward position gain -ve
    // k_r(0,1)= 2*(0.7071); // Yaw gain +ve
    // k_r(0,2)= k_l(0,2) = 1.2*(-16.2331); // Pitch gain -ve
    // k_r(0,3)= k_l(0,3) = (-4.8849); // Forward speed gain -ve
    // k_r(0,4)= (0.4032); // Yaw speed gain +ve
    // k_r(0,5)= k_l(0,5)= 1.5*(-3.1893); // Pitch speed gain -ve
    // k_l(0,1)= -1*k_r(0,1);
    // k_l(0,4)= -1*k_r(0,4);


    // LQR testing
    // k_r(0,0) = 2*-0.5991; // Forward position gain -ve
    // k_l(0,0) = 2*-0.8007;
    // k_r(0,1) = 1.5*0.8007; // Yaw gain +ve
    // k_l(0,1) = 1.5*-0.5991;
    // k_r(0,2) = -15.3053; // Pitch gain -ve
    // k_l(0,2) = -16.2986;
    // k_r(0,3) = 0.85*-4.5527; // Forward speed gain -ve
    // k_l(0,3) = 0.85*-4.4862;
    // k_r(0,4) = 0.3549; // Yaw speed gain +ve
    // k_l(0,4) = -0.5038;
    // k_r(0,5) = -3.0913; // Pitch speed gain -ve
    // k_l(0,5) = -3.2145;
    
    

    ros::Duration(2).sleep(); // Sleep for 2 seconds
    


    CoG_sub = nh.subscribe<geometry_msgs::PointStamped>("/cog/robot",2, CoG_callback);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered",2, odom_callback, ros::TransportHints().tcpNoDelay());
    
    wall_begin = ros::WallTime::now();   
    //ros::spin();
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads for 2 callbacks in parallel
    spinner.spin(); // spin() will not return until the node has been shutdown
  
    

    return 0;
}// end of main