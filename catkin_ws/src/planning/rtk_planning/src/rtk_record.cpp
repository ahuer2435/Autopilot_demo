#include<fstream>

#include <ros/ros.h>
#include <msg_convert/global_pose_vel.h>
#include <tf/transform_datatypes.h>


//#define FILE_NAME "~/workspace/Autopilot_demo/catkin_ws/log/garage.csv"
#define FILE_NAME "./garage.csv"
std::ofstream outfile;

static double_t quad_to_yaw(const geometry_msgs::Quaternion &msg)
{
    tf::Quaternion quad;
    double_t roll, pitch, yaw;

    tf::quaternionMsgToTF(msg,quad);
    tf::Matrix3x3(quad).getRPY(roll, pitch, yaw);

    return yaw;
}

static void global_pose_vel_callback(const msg_convert::global_pose_vel& global_input)
{
    double_t line_x,line_y,line_z,angle_x,angle_y,angle_z;
    double_t latitude = global_input.pose.latitude;
    double_t longitude = global_input.pose.longitude;
    double_t altitude = global_input.pose.altitude;
    double_t yaw = quad_to_yaw(global_input.heading.quaternion);
    line_x = global_input.vel.twist.linear.x;
    line_y = global_input.vel.twist.linear.y;
    line_z = global_input.vel.twist.linear.z;
    angle_x = global_input.vel.twist.angular.x;
    angle_y = global_input.vel.twist.angular.y;
    angle_z = global_input.vel.twist.angular.z;


    outfile << latitude <<"," << longitude <<"," << altitude  <<"," << yaw << "," \
               << line_x <<"," << line_y  <<"," << line_z << "," \
               << angle_x <<"," << angle_y  <<"," << angle_z <<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_record_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("global_pose_vel", 10, global_pose_vel_callback);
    outfile.open(FILE_NAME);
    outfile << "latitude," << "longitude," << "altitude," << "yaw," \
            << "line_x,"  << "line_y," << "line_z," \
            << "angle_x," << "angle_y," << "angle_z" << std::endl;

    ros::spin();
    outfile.close();
    return 0;
}

//refernce: https://blog.csdn.net/CSDNhuaong/article/details/78510436
