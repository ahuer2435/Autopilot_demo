#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <styx_msgs/Lane.h>

ros::Publisher  pub_waypoints;

static void trajectort_convert_callback(const geometry_msgs::PoseArray& trajectort_input)
{
    styx_msgs::Lane lane;
    lane.header = trajectort_input.header;
    for (std::size_t i = 0; i < trajectort_input.poses.size(); ++i) {
      lane.waypoints[i].pose.pose = trajectort_input.poses[i];
      lane.waypoints[i].pose.header = lane.header;
    }
    pub_waypoints.publish(lane);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_convert_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_trajectort = nh.subscribe("trajectory", 10, trajectort_convert_callback);
    pub_waypoints = nh.advertise<styx_msgs::Lane>("final_waypoints", 2, true);

    ros::spin();
    return 0;
}
