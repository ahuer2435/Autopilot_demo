#include<fstream>

#include <ros/ros.h>
#include <msg_convert/global_pose.h>
#include "tf/transform_datatypes.h"


//#define FILE_NAME "~/workspace/Autopilot_demo/catkin_ws/log/garage.csv"
#define FILE_NAME "./garage.csv"
std::ofstream outfile;

static void global_pose_callback(const msg_convert::global_pose& global_input)
{
    tf::Quaternion quad;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(global_input.heading.quaternion,quad);
    double_t latitude = global_input.pose.latitude;
    double_t longitude = global_input.pose.longitude;
    double_t altitude = global_input.pose.altitude;
    tf::Matrix3x3(quad).getRPY(roll, pitch, yaw);
    outfile << latitude <<"," << longitude <<"," << altitude  <<"," << yaw << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_record_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("global_pose", 10, global_pose_callback);
    outfile.open(FILE_NAME);
    outfile << "latitude," << "longitude," << "altitude," << "yaw" << std::endl;
    ros::spin();
    outfile.close();
    return 0;
}

//refernce: https://blog.csdn.net/CSDNhuaong/article/details/78510436
