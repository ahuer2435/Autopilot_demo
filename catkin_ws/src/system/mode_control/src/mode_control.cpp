#include <ros/ros.h>
#include <std_msgs/Bool.h>


ros::Publisher  pub_dbw_enable;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mode_control_node");
    ros::NodeHandle nh;

    std_msgs::Bool dbw_enable;
    pub_dbw_enable = nh.advertise<std_msgs::Bool>("vehicle/dbw_enabled", 2,true);
    dbw_enable.data = true;
    pub_dbw_enable.publish(dbw_enable);
    ros::spin();
    return 0;
}
