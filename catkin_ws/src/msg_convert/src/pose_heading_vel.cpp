#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <msg_convert/global_pose_vel.h>
#include <geometry_msgs/TwistStamped.h>
#include <nmea_navsat_driver/Eulers.h>

ros::Publisher  pub_pose;

typedef boost::shared_ptr< ::nmea_navsat_driver::Eulers const> EularsStampedConstPtr;

void callback(const sensor_msgs::NavSatFixConstPtr &gps_input, const EularsStampedConstPtr &heading_input,const geometry_msgs::TwistStampedConstPtr &vel_input)
{
   msg_convert::global_pose_vel global_pose_vel;
   global_pose_vel.header.stamp = gps_input->header.stamp;
   global_pose_vel.header.frame_id = gps_input->header.frame_id;
   global_pose_vel.pose = *gps_input;
   global_pose_vel.heading = *heading_input;
   global_pose_vel.vel = *vel_input;
   pub_pose.publish(global_pose_vel);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_heading_vel_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "gps", 20);
    message_filters::Subscriber<nmea_navsat_driver::Eulers> heading_sub(nh, "heading", 20);
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, "current_velocity", 20);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, nmea_navsat_driver::Eulers, geometry_msgs::TwistStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, heading_sub, vel_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    pub_pose = nh.advertise< msg_convert::global_pose_vel >("global_pose_vel", 2, true);
    ros::spin();

    return 0;
}
