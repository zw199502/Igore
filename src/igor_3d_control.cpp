#include "igor_3d_control.h"





igor_3d_control::igor_3d_control() //Constructor
{
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",10,&igor_3d_control::imu_callback,this);
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/igor/odom",10,&igor_3d_control::odom_callback,this);
    pub = nh.advertise<geometry_msgs::Vector3>( "/igor/stateVec", 10 );
    cmd_pub1 = nh.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 10 );
    cmd_pub2 = nh.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 10 );
    client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world"); // service client of gazebo service


    k_r(0,0)= k_l(0,0) = -1*(-3.5355);
    k_r(0,1)= -1*(10.1015);
    k_r(0,2)= k_l(0,2) = -1*(-52.5033);
    k_r(0,3)= k_l(0,3) = -1*(-7.8658);
    k_r(0,4)= -1*(1.6642);
    k_r(0,5)= k_l(0,5)= -1*(-17.8761);

    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);

    // k_r(0,0) = -1*-1.0904;
    // k_r(0,1)= -1*0.5119;
    // k_r(0,2)= -1*-21.3701;
    // k_r(0,3)= -1*(-0.8732);
    // k_r(0,4)= -1*(-0.0335);
    // k_r(0,5)= -1*(-8.26);

    // k_l(0,0)=-1*-1.0904;
    // k_l(0,1)=-1*-0.5119;
    // k_l(0,2) = -1*-21.3701; 
    // k_l(0,3) = -1*-0.8732;
    // k_l(0,4)= -1*0.0335;
    // k_l(0,5)= -1*-8.26;

    ref_state(0) = 0.0; // Center Position 
    ref_state(1) = 0.0; // Yaw
    ref_state(2) = 0*0.034; // Pitch
    ref_state(3) = 0.0; // Center velocity 
    ref_state(4) = 0.0; // Yaw velocity
    ref_state(5) = 0.0; // Pitch velocity
        
}

void igor_3d_control::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
        

    igor_orient = msg->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg->angular_velocity;
    //imu_header = msg->header;

    pitch_vel_y = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
    yaw_vel_z = floorf(igor_angul_vel.z*10000)/10000;
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion 
    
    //tf2::convert(igor_orient, quat);// the incoming geometry_msgs::Quaternion is transformed to a tf2::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // the found angles saved in a geometry_msgs::Vector3
    
    



    igor_state(2) = floorf(pitch*10000)/10000;
    igor_state(5) = pitch_vel_y;
    igor_state(1) = floorf(yaw*10000)/10000;
    igor_state(4) = yaw_vel_z;


    this->lqr_controller(igor_state);

    
}// End of imu_callback


void igor_3d_control::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    igor_pose = msg->pose; // igor pose
    igor_twist = msg->twist; // igor twist
    igor_position = igor_pose.pose.position; // igor linear position
    igor_linear_vel = igor_twist.twist.linear; // igor linear velocity
    
    // Publishing robot odometery for rviz
    static tf::TransformBroadcaster br;
    q.setX(igor_pose.pose.orientation.x);
    q.setY(igor_pose.pose.orientation.y);
    q.setZ(igor_pose.pose.orientation.z);
    q.setW(igor_pose.pose.orientation.w);
    transform.setOrigin(tf::Vector3(igor_pose.pose.position.x, igor_pose.pose.position.y, 0.1016));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    /**################################*/

    igor_pos_x = igor_position.x;
    igor_pos_x = floorf(igor_pos_x*1000)/1000;
    // igor_vel_x = (igor_pos_x-igor_last_pos_x)/dt;
    // igor_last_pos_x = igor_pos_x;
    igor_vel_x = igor_linear_vel.x;



    igor_pos_y = (igor_position.y);
    igor_pos_y = floorf(igor_pos_y*1000)/1000;
    igor_vel_y = igor_linear_vel.y;
    // igor_vel_y = (igor_pos_y-igor_last_pos_y)/dt;
    // igor_last_pos_y = igor_pos_y;
    
    trig_vec(0) = cos(floorf(yaw*1000)/1000);
    trig_vec(1) =  sin(floorf(yaw*1000)/1000);
    pos_vec(0,0) = igor_pos_x;
    pos_vec(0,1) = igor_pos_y;
    vel_vec(0,0) = igor_vel_x;
    vel_vec(0,1) = igor_vel_y;

    igor_center_position = (pos_vec*trig_vec).value();
    igor_center_vel = (vel_vec*trig_vec).value();
   
    
    // Filtering of velocity
    vel_filt_in = igor_center_vel;
    igor_center_vel = vel_filt_out = filt1*last_vel_filt_in + filt2*last_vel_filt_out;
    last_vel_filt_out = vel_filt_out;
    last_vel_filt_in = vel_filt_in;
    
    igor_state(0) = igor_center_position;

    igor_state(3) = floorf(igor_center_vel*1000)/1000;

}// End of odom_callback



void igor_3d_control::statePub (geometry_msgs::Vector3 x) // State Vector Publisher
{
        
    pub.publish(x);

}

void igor_3d_control::lqr_controller (Eigen::VectorXf vec) //State-feedback controller
{
    if (igor_state(2)>= -0.3 && igor_state(2) <= 0.3){
        
        igor_3d_control::ref_update();

        trq_r.data = filt_in_r = ((k_r*(vec-ref_state)).value()); // taking the scalar value of the eigen-matrx and filtering it through LPF
        //trq_r.data = filt_out_r = filt1*last_filt_in_r + filt2*last_filt_out_r;
        //last_filt_out_r = filt_out_r;
        //last_filt_in_r = filt_in_r;
        
        trq_l.data = filt_in_l = ((k_l*(vec-ref_state)).value());
        //trq_l.data = filt_out_l = filt1*last_filt_in_l + filt2*last_filt_out_l;
        //last_filt_out_l = filt_out_l;
        //last_filt_in_l = filt_in_l;
 
        cmd_pub1.publish(trq_l);
        cmd_pub2.publish(trq_r);

       
    }
    else if (igor_state(2)<= -1.5 || igor_state(2) >= 1.5){
        trq_r.data = 0;
        trq_l.data = 0;
        cmd_pub1.publish(trq_l);
        cmd_pub2.publish(trq_r);
        ROS_INFO("Reseting Model");
        ros::Duration(0.5).sleep(); // sleep for half a second
        client.call(srv); // Calling the service to reset robot model in gazebo
    }
    

    
    state_vec.x = igor_state(2)-0.034; // Center-position
    state_vec.y = ref_state(1); //Yaw Angle
    state_vec.z = igor_state(3); //Pitch Angle
    this->statePub(state_vec); // Publishing the states

    // ROS_INFO is faster than std::cout
    ROS_INFO("Pitch Angle: %f", igor_state(2)); 
    ROS_INFO("Yaw Angle: %f", igor_state(1)); 
    ROS_INFO("Y Position: %f", igor_pos_y); 
    ROS_INFO("X Position: %f", igor_pos_x); 
    ROS_INFO("Center Position: %f", igor_center_position);
} // End of lqr_controller

void igor_3d_control::ref_update()
{
    //freq = 0.2; // Hz
    //ref_state(0) = igor_state(0)-1; // forward position
    ref_state(1) = 0.78*cos(0.3*ros::Time::now().toSec()); // yaw
    return;
}

igor_3d_control::~igor_3d_control()
{

}


int main(int argc, char **argv){


ros::init(argc, argv, "igor_controller");

igor_3d_control myNode; // creating the igor_control object

ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

//***************************************//

//ros::MultiThreadedSpinner spinner(2); // Use 2 threads for 2 callbacks in parallel
//spinner.spin(); // spin() will not return until the node has been shutdown

//****************************************//

return 0;

} // end of main