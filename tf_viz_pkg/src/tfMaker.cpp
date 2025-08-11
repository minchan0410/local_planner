#include <stdio.h>
#include <cmath>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>

double yaw = 0.0;
double x = 0.0, y = 0.0;

void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    yaw = msg->data;
}

void utmCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped t;

    t.header.stamp = ros::Time::now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link_gps";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 1.7; // GPS 높이

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br.sendTransform(t);
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    double min_dist = 1e9;
    int nearest_idx = -1;

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        double px = msg->poses[i].pose.position.x;
        double py = msg->poses[i].pose.position.y;

        double dist = hypot(px - x, py - y);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = i;
        }
    }


    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped t;

    t.header.stamp = ros::Time::now();
    t.header.frame_id = "map";
    t.child_frame_id = "viewer";

    float x = msg->poses[nearest_idx].pose.position.x;
    float y = msg->poses[nearest_idx].pose.position.y;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    printf("%.2f , %.2f\n",(msg->poses[nearest_idx].pose.position.x / 20 ) * 20, (msg->poses[nearest_idx].pose.position.y / 20 ) * 20);

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br.sendTransform(t);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfmaker");
    ros::NodeHandle nh;

    ros::Subscriber yaw_sub = nh.subscribe("/vehicle_yaw", 1, yawCallback);
    ros::Subscriber utm_sub = nh.subscribe("/odom_gps", 1, utmCallback);
    ros::Subscriber path_sub = nh.subscribe("/global_path", 1, pathCallback);

    ROS_INFO("Map --> Base_link TF Publish Started");

    ros::spin();
    return 0;
}