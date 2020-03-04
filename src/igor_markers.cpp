#include "igor_markers.h"


igor_markers::igor_markers() //Constructor
{

    center_frame = nh_.subscribe<nav_msgs::Odometry>("/igor/center",1, & igor_markers::ref_frame_callback, this);
    base_frame = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, & igor_markers::support_line, this);
    ref_marker_pub = nh_.advertise<visualization_msgs::Marker>("ref_marker", 1);
    support_marker_pub = nh_.advertise<visualization_msgs::Marker>("support_marker", 1);


} // end of constructor


void igor_markers::ref_frame_callback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    igor_pose = msg->pose; // igor pose
    igor_position = igor_pose.pose.position; // igor linear position



    ref_marker.header.frame_id = "/map";
    ref_marker.header.stamp = ros::Time::now();
    ref_marker.ns = "sphere_shape";
    ref_marker.id = 0;
    ref_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    ref_marker.action = visualization_msgs::Marker::ADD;
    // ref_marker.pose.position.x = igor_position.x;
    // ref_marker.pose.position.y = igor_position.y;
    // ref_marker.pose.position.z = igor_position.z;
    ref_marker.points.push_back(igor_position);
    ref_marker.pose.orientation.x = 0;
    ref_marker.pose.orientation.y = 0;
    ref_marker.pose.orientation.z = 0;
    ref_marker.pose.orientation.w = 1;
    ref_marker.scale.x = 0.05;
    ref_marker.scale.y = 0.05;
    ref_marker.scale.z = 0.05;
    ref_marker.color.r = 0.078;
    ref_marker.color.g = 1;
    ref_marker.color.b = 0.855;
    ref_marker.color.a = 1.0;
    ref_marker.lifetime = ros::Duration();
    ros::Duration(0.2).sleep();
    ref_marker_pub.publish(ref_marker);

} // end of ref_frame_callback

void igor_markers::support_line(const nav_msgs::Odometry::ConstPtr &msg){

    try
    { 
        transformStampedL = tfBufferL.lookupTransform("base_link", "L_wheel" , ros::Time(0));
        transformStampedR = tfBufferR.lookupTransform("base_link", "R_wheel" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    Lwheel_position.x = transformStampedL.transform.translation.x;
    Lwheel_position.y = transformStampedL.transform.translation.y;
    Lwheel_position.z = transformStampedL.transform.translation.z;

    Rwheel_position.x = transformStampedR.transform.translation.x;
    Rwheel_position.y = transformStampedR.transform.translation.y;
    Rwheel_position.z = transformStampedR.transform.translation.z;


    support_line_marker.header.frame_id = "/base_link";
    support_line_marker.header.stamp = ros::Time::now();
    support_line_marker.ns = "line_shape";
    support_line_marker.id = 1;
    support_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    support_line_marker.action = visualization_msgs::Marker::ADD;
    support_line_marker.pose.orientation.w = 1;
    support_line_marker.points.clear();
    support_line_marker.points.push_back(Lwheel_position);
    support_line_marker.points.push_back(Rwheel_position);
    support_line_marker.scale.x = 0.04;
    support_line_marker.color.r = 0.224;
    support_line_marker.color.g = 1;
    support_line_marker.color.b = 0.078;
    support_line_marker.color.a = 1.0;
    support_line_marker.lifetime = ros::Duration(0);
    support_marker_pub.publish(support_line_marker);

} // end of support_line



igor_markers::~igor_markers(){

} // end of destructor



int main(int argc, char **argv){


ros::init(argc, argv, "igor_markers");

igor_markers igor_marker_node; // creating the igor_markers object

ros::Duration(0.1).sleep();
//ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).
ros::MultiThreadedSpinner spinner(2); // Use 2 threads for 2 callbacks in parallel
spinner.spin(); // spin() will not return until the node has been shutdown


return 0;

} // end of main