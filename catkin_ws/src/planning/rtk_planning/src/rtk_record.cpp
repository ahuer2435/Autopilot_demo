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
    double_t x = global_input.pose.pose.position.x;
    double_t y = global_input.pose.pose.position.y;
    double_t z = global_input.pose.pose.position.z;
    double_t yaw = global_input.heading.yaw;
    line_x = global_input.vel.twist.linear.x;
    line_y = global_input.vel.twist.linear.y;
    line_z = global_input.vel.twist.linear.z;
    angle_x = global_input.vel.twist.angular.x;
    angle_y = global_input.vel.twist.angular.y;
    angle_z = global_input.vel.twist.angular.z;

    //printf("global_input.pose.pose.position.z = %lf\n",global_input.pose.pose.position.z);
    outfile << x <<"," << y <<"," << z  <<"," << yaw << "," \
               << line_x <<"," << line_y  <<"," << line_z << "," \
               << angle_x <<"," << angle_y  <<"," << angle_z <<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_record_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("pose_heading_vel", 10, global_pose_vel_callback);
    outfile.open(FILE_NAME);
    outfile << "x," << "y," << "z," << "yaw," \
            << "line_x,"  << "line_y," << "line_z," \
            << "angle_x," << "angle_y," << "angle_z" << std::endl;

    ros::spin();
    outfile.close();
    return 0;
}

//refernce: https://blog.csdn.net/CSDNhuaong/article/details/78510436
