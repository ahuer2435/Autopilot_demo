#include <ros/ros.h>
#include <msg_convert/global_pose.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher  pub_current_pose;

static void trajectort_convert_callback(const msg_convert::global_pose& global_pose_input)
{
    //TODO
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_global_current_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_global_pose = nh.subscribe("global_pose", 10, trajectort_convert_callback);
    pub_current_pose = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 2, true);

    ros::spin();
    return 0;
}
