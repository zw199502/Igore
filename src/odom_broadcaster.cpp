#include "odom_broadcaster.h"


odom_broadcaster::odom_broadcaster(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &odom_broadcaster::odom_callback,this);
} // End of constructor

void odom_broadcaster::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    igor_robot_pose_ = msg->pose; // robot pose

    transform_.header.stamp = msg->header.stamp;
    transform_.header.frame_id = "map";
    transform_.child_frame_id = "base_link";
    transform_.transform.translation.x = igor_robot_pose_.pose.position.x;
    transform_.transform.translation.y = igor_robot_pose_.pose.position.y;
    transform_.transform.translation.z = igor_robot_pose_.pose.position.z;
    transform_.transform.rotation.x = igor_robot_pose_.pose.orientation.x;
    transform_.transform.rotation.y = igor_robot_pose_.pose.orientation.y;
    transform_.transform.rotation.z = igor_robot_pose_.pose.orientation.z;
    transform_.transform.rotation.w = igor_robot_pose_.pose.orientation.w;


    br.sendTransform(transform_);
    

}// End of odom_callback

odom_broadcaster::~odom_broadcaster()
{

} // End of destructor







int main(int argc, char **argv){


ros::init(argc, argv, "odom_broadcaster");
ros::NodeHandle nh;
odom_broadcaster my_odom_broadcaster_node(&nh); // creating the odom_broadcaster object

ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

return 0;

} // end of main