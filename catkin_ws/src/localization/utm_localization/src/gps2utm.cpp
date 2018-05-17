#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include "gps_to_utm.h"

tf2::Transform transform_utm_pose_;
std::string gps_frame_id_;
Eigen::MatrixXd latest_utm_covariance_;
tf2::Transform latest_utm_pose_;
tf2::Transform last_utm_pose_;
bool transform_good_ = false;
bool last_utmX_set = false;
bool last_utmY_set = false;
std::string utm_zone_;

const int POSITION_SIZE = 3;

typedef struct utm_struct{
    double utm_x;
    double utm_y;
}utm_data;

utm_data curr_utm;
utm_data last_utm;
double yaw, roll, pitch;

std::string MAP_FRAME_ = "utm";
std::string GPS_FRAME_ = "gps";

ros::Publisher pub1_;

//前后两个位置的夹角.
void createOrientation(utm_data last_utm, utm_data curr_utm)
{
  yaw = atan2(curr_utm.utm_x - last_utm.utm_x, curr_utm.utm_y - last_utm.utm_y);
  roll = 0;
  pitch = 0;
}

void publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  createOrientation(last_utm,curr_utm);
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = curr_utm.utm_y;
  pose.pose.position.y = curr_utm.utm_x;
  pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  pub1_.publish(pose);
}

//发布MAP_FRAME_和GPS_FRAME_的转化tf关系.
void publishTF()
{
    static tf2_ros::TransformBroadcaster br_;           //如果没有声明为static,就不会出现/tf话题.重点mark下,这个问题,调试了蛮久
    geometry_msgs::TransformStamped transformStamped;   //且如果没有实际数据进来,也不会出现/tf话题.
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = MAP_FRAME_;
    transformStamped.child_frame_id = GPS_FRAME_;

    transformStamped.transform.translation.x = transform_utm_pose_.getOrigin().getX();
    transformStamped.transform.translation.y = transform_utm_pose_.getOrigin().getY();
    transformStamped.transform.translation.z = transform_utm_pose_.getOrigin().getZ();

    transformStamped.transform.rotation.x = transform_utm_pose_.getRotation().getX();
    transformStamped.transform.rotation.y = transform_utm_pose_.getRotation().getY();
    transformStamped.transform.rotation.z = transform_utm_pose_.getRotation().getZ();
    transformStamped.transform.rotation.w = transform_utm_pose_.getRotation().getW();
    br_.sendTransform(transformStamped);

}

//将gps 位姿数据转化为utm位姿数据，gps位姿数据设置为utm坐标系的原点和朝向
  void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double utm_x = 0;
    double utm_y = 0;
    LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone_);

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " <<
                    msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utm_x << ", " << utm_y << ")");

    transform_utm_pose_.setOrigin(tf2::Vector3(utm_x, utm_y, 0));
    transform_utm_pose_.setRotation(tf2::Quaternion::getIdentity());
  }


void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  gps_frame_id_ = msg->header.frame_id;     //gps消息frame_id设置gps_frame_id_。

  if (gps_frame_id_.empty())
  {
    ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's " "origin.");
  }

  // Make sure the GPS data is usable
  bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&  !std::isnan(msg->latitude) && !std::isnan(msg->longitude));

  if (good_gps)
  {

    if (!transform_good_)
    {
      setTransformGps(msg);
      transform_good_ = true;
    }

    std::string utm_zone_tmp;
    LLtoUTM(msg->latitude, msg->longitude, curr_utm.utm_y, curr_utm.utm_x, utm_zone_tmp);

    latest_utm_pose_.setOrigin(tf2::Vector3(curr_utm.utm_x, curr_utm.utm_y, msg->altitude));
    latest_utm_covariance_.setZero();
#if 0   //这里运行出错,暂时没有找到原因.现行注掉.

    for (size_t i = 0; i < POSITION_SIZE; i++)
    {
      for (size_t j = 0; j < POSITION_SIZE; j++)
      {
          std::cout<<"i="<<i<<"\n"<<"j="<<j<<std::endl;
        latest_utm_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
      }
    }
#endif


    last_utm.utm_x = curr_utm.utm_x;
    last_utm.utm_y = curr_utm.utm_y;

    publishTF();
    publishPoseStamped();

  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps2utm_node");
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("gps", 1,gpsFixCallback);
    pub1_ = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
    ros::spin();
    return 0;
}

