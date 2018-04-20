#include <ros/ros.h>
#include <msg_convert/global_pose_vel.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher  pub_current_pose;


static void trajectort_convert_callback(const msg_convert::global_pose_vel& global_pose_input)
{
    double northing,easting;
    std::string zone;
    geometry_msgs::PoseStamped geo_pose;

    geo_pose.header = global_pose_input.pose.header;
    geo_pose.pose.position.x = global_pose_input.pose.pose.position.x;
    geo_pose.pose.position.y = global_pose_input.pose.pose.position.y;
    geo_pose.pose.position.z = global_pose_input.pose.pose.position.z;
    geo_pose.pose.orientation = global_pose_input.pose.pose.orientation;
    pub_current_pose.publish(geo_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_global_current_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_global_pose = nh.subscribe("global_pose_vel", 10, trajectort_convert_callback);
    pub_current_pose = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 2, true);

    ros::spin();
    return 0;
}
