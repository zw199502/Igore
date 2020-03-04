#include <igor/igor_states.h>


ros::ServiceClient client;
std_srvs::Empty srv;
ros::Publisher  cmd_pub1; // creating ROS publisher
ros::Publisher  cmd_pub2; // creating ROS publisher
Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
Eigen::VectorXf ref_state = Eigen::VectorXf(6);
std_msgs::Float64 trq_r;
std_msgs::Float64 trq_l;


void lqr_controller()
{

    if (igor_state(2)>= -0.35 && igor_state(2) <= 0.35)
    {
        trq_r.data =  ((k_r*(igor_state-ref_state)).value()); // taking the scalar value of the eigen-matrx   
        trq_l.data =  ((k_l*(igor_state-ref_state)).value());
        cmd_pub1.publish(trq_l);
        cmd_pub2.publish(trq_r);
    }
    else if (igor_state(2)<= -1.4 || igor_state(2) >= 1.4){
        trq_r.data = 0;
        trq_l.data = 0;
        cmd_pub1.publish(trq_l);
        cmd_pub2.publish(trq_r);
        ROS_INFO("Reseting Model");
        ros::Duration(0.5).sleep(); // sleep for half a second
        client.call(srv); // Calling the service to reset robot model in gazebo
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_main");
    ros::NodeHandle nh;
    cmd_pub1 = nh.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 1 ); // creating ROS publisher
    cmd_pub2 = nh.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 1 ); // creating ROS publisher
    client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world"); // service client of gazebo service
    k_r(0,0)= k_l(0,0) = -1*(-3.5355);
    k_r(0,1)= -1*(10.1015);
    k_r(0,2)= k_l(0,2) = -1*(-51.1363);
    k_r(0,3)= k_l(0,3) = -1*(-7.9125);
    k_r(0,4)= -1*(1.6642);
    k_r(0,5)= k_l(0,5)= -1*(-16.9210);
    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);
    ref_state(0) = 0.0; // Center Position 
    ref_state(1) = 0.0; // Yaw
    ref_state(2) = 0.0; // Pitch
    ref_state(3) = 0.0; // Center Velocity
    ref_state(4) = 0.0; // Yaw velocity
    ref_state(5) = 0.0; // Pitch velocity

    igor_states myNode(&nh);

    ros::Rate rate_timer(500); //Loop rate in Hz
    while (ros::ok())
    {

        lqr_controller();
        ros::spinOnce();
        rate_timer.sleep();
    }
    
    return 0;
}
