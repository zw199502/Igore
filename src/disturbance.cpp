#include "disturbance.h"

/* This Node applies wrench on a slected link of the robot by calling gazebo/ApplyBodyWrench service.
*/

disturbance::disturbance() //Constructor
{
    sub_clk = nh_.subscribe<rosgraph_msgs::Clock>("/clock",10,&disturbance::clk_callback,this);
    client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench"); // service client of gazebo service
    igor_wrench.force.x = 0; // Force in newtons
    igor_wrench.force.y = 0;
    igor_wrench.force.z = 0;
    igor_wrench.torque.x = 0; // Moment in Nm
    igor_wrench.torque.y = 0;
    igor_wrench.torque.z = 0;

    srv.request.body_name = igor_body_name;
    srv.request.reference_frame = igor_reference_frame;
    srv.request.wrench = igor_wrench;
    srv.request.duration = ros::Duration(1); // set duration of wrench to 1 sec
}

void disturbance::clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg){

    my_time = msg->clock;

    //ROS_INFO("Simulation Time %f", my_time.toSec());
    
    if (my_time.toSec()==8){ // To call the rosservice at the 8th sec
        if(!run){
            ROS_INFO("Calling Apply_Body_Wrench Service");
            client.call(srv); // call the service
            run = true; // Run only once
        }

    }

} // end of clk_callback

disturbance::~disturbance(){

}

int main(int argc, char **argv){


ros::init(argc, argv, "disturber");

disturbance my_disturbance_node; // creating the disturbance object

ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

return 0;

} // end of main