#include "igor_control.h"





igor_control::igor_control() //Constructor
{
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/igor/imu/data",10,&igor_control::imu_callback,this);
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/igor/odom",10,&igor_control::odom_callback,this);
    // sync_imu.subscribe(nh,"/igor/imu/data",50);
    // sync_odom.subscribe(nh,"/igor/odom",50);
    // sync_.reset(new sync(sync_imu, sync_odom, 50));
    // sync_.reset(new Sync(MySyncPolicy(50), sync_imu, sync_odom));
    // sync_->registerCallback(boost::bind(&igor_control::sync_callback, this, _1, _2));
    pub = nh.advertise<geometry_msgs::Vector3>( "/igor/stateVec", 10 );
    cmd_pub1 = nh.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 10 );
    cmd_pub2 = nh.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 10 );
    //cmd_pub1 = nh.advertise<geometry_msgs::Twist>( "/igor/diff_drive_controller/cmd_vel", 10 );
    client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world"); // service client of gazebo service

    k_vec(0,0)= 2.8;
    k_vec(0,1)= -1*(-25.3935);
    k_vec(0,2)= 11.0988;
    k_vec(0,3)= -1*(-18.6061);

    k_lqr(0,0)= 2.5;
    k_lqr(0,1)= -1*(-39.7198);
    k_lqr(0,2)= 9.9031;
    k_lqr(0,3)= -1*(-11.2715);

    ref_state(0) = 0.0+0.47; // Position X
    ref_state(1) = 0.0; // Pitch theta
    ref_state(2) = 0.0; // Velocity X_dot
    ref_state(3) = 0.0; // Angular velocity theta_dot
        
}
//*********NOT USING*****************
void igor_control::sync_callback(const sensor_msgs::Imu::ConstPtr &msg1, const nav_msgs::Odometry::ConstPtr &msg2)
{
    
    igor_orient = msg1->orientation;
    igor_angul_vel = msg1->angular_velocity;
    //imu_header = msg->header;

    ang_vel_y = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
        
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
      
    tf::quaternionMsgToTF(igor_orient, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // the found angles saved in a geometry_msgs::Vector3
    
    //rpy.x = roll;
    //rpy.y = pitch;
    //rpy.z = yaw;
    
    
    ang_accl_y = (ang_vel_y-last_ang_vel)/dt; // Angular accel. about Y-axis in rad/sec^2
    ang_accl_y = floorf(ang_accl_y*10000)/10000; // round off data upto 4 decimal points
    
    last_ang_vel = ang_vel_y;
    


    eig_vec(0) = (ang_accl_y); // assigning value to state vector (Eigen Class)
    eig_vec(1) = (ang_vel_y);
    eig_vec(2) = floorf(pitch*1000)/1000; // round off data upto 3 decimal points

    igor_state(1) = -1*floorf(pitch*10000)/10000;
    igor_state(3) = -1*(ang_vel_y);

    error = sp1+floorf(pitch*100000)/100000; // positive feedback system
    error_d = (error-last_err)/dt;
    last_err = error;
    error_i += error*dt;


    igor_odom = msg2->pose;
    igor_position = igor_odom.pose.position;
    igor_pos_x = (igor_position.x);
    igor_vel_x = (igor_pos_x-igor_last_pos)/dt;
    igor_last_pos = igor_pos_x;
    
    vel_filt_in = igor_vel_x;
    igor_vel_x = vel_filt_out = filt1*last_vel_filt_in + filt2*last_vel_filt_out;
    last_vel_filt_out = vel_filt_out;
    last_vel_filt_in = vel_filt_in;
    
    igor_state(0) = 1*floorf(igor_pos_x*1000)/1000;

    igor_state(2) = 1*floorf(igor_vel_x*1000)/1000;

 
    //ROS_INFO("Sync done");
    ROS_INFO("State 1: %f", igor_state(0));
    ROS_INFO("State 2: %f", igor_state(1)); // ROS_INFO is faster than std::cout
    this->sf_controller(igor_state);
    //this->PID_controller();

}
//***********************************

void igor_control::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
        

    igor_orient = msg->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg->angular_velocity;
    //imu_header = msg->header;

    ang_vel_y = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion 
    
    //tf2::convert(igor_orient, quat);// the incoming geometry_msgs::Quaternion is transformed to a tf2::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // the found angles saved in a geometry_msgs::Vector3
    
    //rpy.x = roll;
    //rpy.y = pitch;
    //rpy.z = yaw;
    
    
    ang_accl_y = (ang_vel_y-last_ang_vel)/dt; // Angular accel. about Y-axis in rad/sec^2
    //ang_accl_y = floorf(ang_accl_y*10000)/10000; // round off data upto 4 decimal points
    last_ang_vel = ang_vel_y;
    


    // eig_vec(0) = (ang_accl_y); // assigning value to state vector (Eigen Class)
    // eig_vec(1) = (ang_vel_y);
    // eig_vec(2) = floorf(pitch*10000)/10000; // round off data upto 4 decimal points

    igor_state(1) = 1*(floorf(pitch*10000)/10000);
    igor_state(3) = 1*(ang_vel_y);


    this->sf_controller(igor_state);
    //this->PID_controller();
    //this->Poly_PID(); 

    
}// End of imu_callback


void igor_control::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    igor_odom = msg->pose;
    igor_position = igor_odom.pose.position;
    igor_pos_x = (igor_position.x);
    igor_vel_x = (igor_pos_x-igor_last_pos)/dt;
    igor_last_pos = igor_pos_x;
    
    vel_filt_in = igor_vel_x;
    igor_vel_x = vel_filt_out = filt1*last_vel_filt_in + filt2*last_vel_filt_out;
    last_vel_filt_out = vel_filt_out;
    last_vel_filt_in = vel_filt_in;
    
    igor_state(0) = 1*floorf(igor_pos_x*10000)/10000;

    igor_state(2) = 1*floorf(igor_vel_x*10000)/10000;

    //ROS_INFO("In Odom");
    //return;
}// End of odom_callback



void igor_control::statePub (geometry_msgs::Vector3 x) // State Vector Publisher
{
        
    pub.publish(x);

}

void igor_control::sf_controller (Eigen::Vector4f vec) //State-feedback controller
{
    if (igor_state(1)>= -0.3 && igor_state(1) <= 0.3){
        

        //ref_state(0) = igor_state(0) + (3);
        
        trq.data = filt_in = 0.5*((k_lqr*(vec-ref_state)).value()); // taking the scalar value of the eigen-matrx and filtering it through LPF
        //trq.data = filt_out = filt1*last_filt_in + filt2*last_filt_out;
        //last_filt_out = filt_out;
        //last_filt_in = filt_in;

        if (trq.data > 3){
        trq.data = 3;
        }
        else if (trq.data <-3){
            trq.data = -3;
        }

        // accl = trq.data / whl_Inertia;
        // omga += accl*dt;
        // lin_vel = wheel_rad * omga;
        // vel_cmnd.linear.x = lin_vel;
    }
    else if (igor_state(1)<= -1.5 || igor_state(1) >= 1.5){
        trq.data = 0;
        cmd_pub1.publish(trq);
        cmd_pub2.publish(trq);
        ROS_INFO("Reseting Model");
        ros::Duration(0.5).sleep(); // sleep for half a second
        client.call(srv); // Calling the service to reset robot model in gazebo
    }
    
    if ( abs(igor_state(0)) <=3 ){
        cmd_pub1.publish(trq);
        cmd_pub2.publish(trq);
    }
    else if ( abs(igor_state(0)) >3){
        cmd_pub1.publish(trq);
        //trq.data = 0.5*(trq.data);
        cmd_pub2.publish(trq);
        //ROS_INFO("Moving right");
    }

    
    state_vec.x = igor_state(0); // X-position
    state_vec.y = igor_state(1)-0.034; //Pitch Angle
    state_vec.z = -2*trq.data;
    this->statePub(state_vec); // Publishing the states
    //std::cout<< "State Vector = " << state_vec << std::endl<< "Torque Command = " << trq.data << std::endl;
    ROS_INFO("Torque: %f", trq.data);
    ROS_INFO("State 2: %f", state_vec.y); // ROS_INFO is faster than std::cout
}

void igor_control::PID_controller() //PID controller
{
    error = sp1+floorf(pitch*100000)/100000; // positive feedback system
    error_d = (error-last_err)/dt;
    last_err = error;
    error_i += error*dt;
    
    trq.data = filt_in = 0.5*((error*kp)+(error_d*kd)+(error_i*ki));
    // trq.data = filt_out = filt1*last_filt_in + filt2*last_filt_out;
    // last_filt_out = filt_out;
    // last_filt_in = filt_in;

    cmd_pub1.publish(trq);
    cmd_pub2.publish(trq);
    
    state_vec.x = igor_state(1);
    state_vec.y = trq.data;
    state_vec.z = igor_state(0);
    this->statePub(state_vec);
    std::cout<< "State Vector = " << state_vec << std::endl<< "Torque Command = " << trq.data << std::endl;


}


void igor_control::Poly_PID()
{
    PD_out = k_p*igor_state(1) + k_d*igor_state(3);
    sys_in = sp2 - PD_out;
    trq.data = -1*sys_in;
    cmd_pub1.publish(trq);
    cmd_pub2.publish(trq);

    state_vec.x = igor_state(0);
    state_vec.y = -igor_state(1);
    state_vec.z = trq.data;
    this->statePub(state_vec);
    std::cout<< "State Vector = " << state_vec << std::endl<< "Torque Command = " << trq.data << std::endl;

}

igor_control::~igor_control()
{

}


int main(int argc, char **argv){


ros::init(argc, argv, "igor_controller");

igor_control myNode; // creating the igor_control object

ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

//ros::MultiThreadedSpinner spinner(2); // Use 2 threads for 2 callbacks in parallel
//spinner.spin(); // spin() will not return until the node has been shutdown

return 0;

} // end of main