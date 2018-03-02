#include<fstream>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

//#define FILE_NAME "~/workspace/Autopilot_demo/catkin_ws/log/garage.csv"
#define FILE_NAME "./garage.csv"
std::ofstream outfile;

static void gps_callback(const sensor_msgs::NavSatFix& gps_input)
{
    double_t latitude = gps_input.latitude;
    double_t longitude = gps_input.longitude;
    double_t altitude = gps_input.altitude;
    outfile << latitude <<"," << latitude <<"," << latitude << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_record_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("gps", 10, gps_callback);
    outfile.open(FILE_NAME);
    outfile << "latitude," << "latitude," << "latitude" << std::endl;
    ros::spin();
    outfile.close();
    return 0;
}

