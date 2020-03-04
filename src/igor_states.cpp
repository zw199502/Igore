#include <igor/igor_states.h>





igor_states::igor_states(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{
    sub_center_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/center_imu/data",1, &igor_states::center_imu_callback,this);
    //sub_joint_states = nh_.subscribe<sensor_msgs::JointState>("/igor/joint_states",1, &igor_states::joint_states_callback,this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &igor_states::odom_callback,this);
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_states::CoG_callback,this);
    

    unit_vec << 0, 0, 1;
    tf::vectorEigenToTF(unit_vec,unit_tf);
    ref_origin.x = ref_origin.y = ref_origin.z = 0;

        
} // End of constructor

void igor_states::center_imu_callback(const sensor_msgs::Imu::ConstPtr &msg1)
{
        

    igor_orient = msg1->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg1->angular_velocity;

    //pitch_vel_y = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
    yaw_vel_z = floorf(igor_angul_vel.z*10000)/10000;
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    //the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //the found angles saved in a geometry_msgs::Vector3
    
    
    //igor_state(2) = floorf(pitch*10000)/10000;
    //igor_state(5) = pitch_vel_y;
    
    igor_state(1) = floorf(yaw*10000)/10000;
    igor_state(4) = yaw_vel_z;



    
}// End of imu_callback


void igor_states::odom_callback(const nav_msgs::Odometry::ConstPtr &msg3)
{

    igor_pose = msg3->pose; // igor pose
    igor_twist = msg3->twist; // igor twist
    igor_position = igor_pose.pose.position; // igor linear position
    igor_linear_vel = igor_twist.twist.linear; // igor linear velocity
    

    igor_pos_x = igor_position.x;
    igor_pos_x = floorf(igor_pos_x*1000)/1000;
    igor_vel_x = igor_linear_vel.x;  

    igor_pos_y = (igor_position.y);
    igor_pos_y = floorf(igor_pos_y*1000)/1000;
    igor_vel_y = igor_linear_vel.y;
    
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

// void igor_states::joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg2) // Joint State Subscriber
// {
//     joint_pos = msg2->position;
//     joint_vel = msg2->velocity;

//     L_knee_pos = joint_pos[1];
//     L_knee_vel = joint_vel[1];
//     R_knee_pos = joint_pos[4];
//     R_knee_vel = joint_vel[4];

//     //igor_state(3) = floorf(L_knee_pos*10000)/10000;
//     //igor_state(7) = floorf(L_knee_vel*10000)/10000;



// }// End of joint_states_callback

void igor_states::CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg4)
{

    CoG_Position = msg4->point;
  
    CoG_angle = atan2(CoG_Position.x, CoG_Position.z) + pitch;
    CoG_angle = floorf(CoG_angle*10000)/10000;
    igor_state(2) = CoG_angle;

    now = msg4->header.stamp;
    dt = now-last_time;
    if(dt.toSec() > 0.0)
    {
        CoG_pitch_vel = (CoG_angle-CoG_last_angle)/dt.toSec();
    }
    else
    {   
        ROS_WARN_STREAM("Duration dt <= 0.0");
        CoG_pitch_vel = CoG_last_pitch_vel;
    }
    
    //ROS_INFO("CoG last Angle: %f", CoG_last_angle);

    igor_state(5) = CoG_pitch_vel;
    
    last_time = now;
    CoG_last_angle = CoG_angle;
    CoG_last_pitch_vel = CoG_pitch_vel;

    /**###############################################**/

    try
    { 
        now2 = ros::Time::now();
        if (tfBuffer.canTransform("map", "robot_center_link", now2, ros::Duration(0.0001))){
            transformStamped = tfBuffer.lookupTransform("map", "robot_center_link" , now2);
        }
        else{
            ROS_WARN_STREAM("No tf2 Transform");
        }  
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }



    robot_center_pos << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z;
    tf2::doTransform(CoG_Position, CoG_Position, transformStamped);
    CoM_pos << CoG_Position.x, CoG_Position.y, CoG_Position.z;

    

    CoM_vec = (CoM_pos - robot_center_pos);
    tf::vectorEigenToTF(CoM_vec,CoM_tf); // converting Eigen Vector to tf Vector
    
    // if (CoM_vec.dot(unit_vec)<0){
    //     CoG_angle3 = -acos((CoM_vec.dot(unit_vec))/(CoM_vec.norm()*unit_vec.norm()));
    // }
    // else{
    //     CoG_angle3 = acos((CoM_vec.dot(unit_vec))/(CoM_vec.norm()*unit_vec.norm()));
    // }

    CoM_tf.normalize(); 
    CoG_angle2 = atan2(rot_axis.dot(unit_tf.cross(CoM_tf)), CoM_tf.dot(unit_tf));
    
    
    
    CoG_angle2 = floorf(CoG_angle2*10000)/10000;



    //now2 = ros::Time::now().toSec(); 
    //now2 = transformStamped.header.stamp;
    dt2 = (now2-last_time2);
    if(dt2.toSec() > 0.0)
    {
        CoG_pitch_vel2 = (CoG_angle2 - CoG_last_angle2)/dt2.toSec();
    }
    else
    {   
        ROS_WARN_STREAM("Duration dt2 <= 0.0");
        ROS_INFO("dt2 = %f", dt2.toSec());
        CoG_pitch_vel2 = CoG_last_pitch_vel2;
    }
    //igor_state(2) = CoG_angle2;
    //igor_state(5) = CoG_pitch_vel2;
    
    //ROS_INFO("CoG Angle: %f", CoG_angle);
    //ROS_INFO("CoG Angle2: %f", CoG_angle2);

    last_time2 = now2;
    CoG_last_angle2 = CoG_angle2;
    CoG_last_pitch_vel2 = CoG_pitch_vel2;

    //ROS_INFO("CoG Pitch Vel: %f", CoG_pitch_vel);
    //ROS_INFO("CoG Pitch Vel2: %f", CoG_pitch_vel2);
    //ROS_INFO("Time period dt: %f", dt);
    //ROS_INFO("Time period dt2: %f", dt2);

} // End of CoG_callback




igor_states::~igor_states()
{

} // End of destructor