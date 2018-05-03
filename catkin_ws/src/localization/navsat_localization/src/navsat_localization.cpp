#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include "navsat_localization.h"

ros::Publisher pub_pose;
static void gps_callback(const sensor_msgs::NavSatFix& global_pose)
{
    geometry_msgs::PoseStamped pose;
    double northing,easting;
    std::string zone;
    LLtoUTM(global_pose.latitude,global_pose.longitude,northing,easting,zone);

    pose.header.frame_id = "/utm";
    pose.header.stamp = global_pose.header.stamp;
    pose.pose.position.x = easting;
    pose.pose.position.y = northing;
    if(isnan(global_pose.altitude)){
        pose.pose.position.z = 0.0;
        //printf("isnan global_pose.altitude = %lf\n",global_pose.altitude);
    }else{
        pose.pose.position.z = global_pose.altitude;
        //printf("isnot nan global_pose.altitude = %lf\n",global_pose.altitude);
    }

    pub_pose.publish(pose);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsat_localization_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("gps", 1, gps_callback);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("localization/pose",1,true);

    ros::spin();

    return 0;
}

//refernce: https://blog.csdn.net/CSDNhuaong/article/details/78510436
