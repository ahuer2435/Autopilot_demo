#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <msg_convert/global_pose.h>


ros::Publisher  pub_pose;

void callback(const sensor_msgs::NavSatFixConstPtr &gps_input, const geometry_msgs::QuaternionStampedConstPtr &heading_input)
{
   msg_convert::global_pose global_pose;
   global_pose.header.stamp = gps_input->header.stamp;
   global_pose.header.frame_id = gps_input->header.frame_id;
   global_pose.pose = *gps_input;
   global_pose.heading = *heading_input;
   pub_pose.publish(global_pose);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_heading_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "gps", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> heading_sub(nh, "heading", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, heading_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub_pose = nh.advertise< msg_convert::global_pose >("global_pose", 2, true);
    ros::spin();

    return 0;
}
