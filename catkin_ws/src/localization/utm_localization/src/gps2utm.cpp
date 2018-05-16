#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/NavSatFix.h>
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
ros::Time gps_update_time_;
bool gps_updated_;
std::string utm_zone_;
bool has_transform_gps_;

const int POSITION_SIZE = 3;

typedef struct utm_struct{
    double utm_x;
    double utm_y;
}utm_data;

//前后两个位置的夹角.
void createOrientation(utm_data last_utm, utm_data curr_utm, double_t &roll,double_t &pitch,double_t &yaw)
{
  yaw_ = atan2(curr_utm.utm_x - last_utm.utm_x, curr_utm.utm_y - last_utm.utm_y);
  roll_ = 0;
  pitch_ = 0;
}


void NavSatTransform::computeTransform()
{
  // Only do this if:
  // 1. We haven't computed the odom_frame->utm_frame transform before
  // 2. We've received the data we need
  if (!transform_good_  && has_transform_gps_ )
  {
    // The UTM pose we have is given at the location of the GPS sensor on the robot. We need to get the UTM pose of
    // the robot's origin.
      //transform_utm_pose_是gps在utm坐标系下的位置,transform_utm_pose_corrected是robot在utm坐标系下的位置.
    tf2::Transform transform_utm_pose_corrected;
    if (!use_manual_datum_)
    {
      getRobotOriginUtmPose(transform_utm_pose_, transform_utm_pose_corrected, ros::Time(0));
    }
    else
    {
      transform_utm_pose_corrected = transform_utm_pose_;
    }

    // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
    tf2::Matrix3x3 mat(transform_orientation_);

    // Convert to RPY
    double imu_roll;
    double imu_pitch;
    double imu_yaw;
    mat.getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_yaw += (magnetic_declination_ + yaw_offset_);

    ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magnetic_declination_ <<
                    " and user-specified offset of " << yaw_offset_ << "." <<
                    " Transform heading factor is now " << imu_yaw);

    // Convert to tf-friendly structures
    tf2::Quaternion imu_quat;
    imu_quat.setRPY(0.0, 0.0, imu_yaw);

    // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
    // Doing it this way will allow us to cope with having non-zero odometry position
    // when we get our first GPS message.
    //utm_pose_with_orientation是在utm坐标系下的位置和朝向.
    tf2::Transform utm_pose_with_orientation;
    utm_pose_with_orientation.setOrigin(transform_utm_pose_corrected.getOrigin());
    utm_pose_with_orientation.setRotation(imu_quat);

    //transform_world_pose_,其保存的是world_frame_id_(odom)和base_link_frame_id_(base_link)坐标系之间的关系
    //utm_pose_with_orientation保存的是utm坐标系和base_link_frame_id_(base_link)坐标系之间的关系
    //utm_world_transform_保存的是world_frame_id_(odom)和utm坐标系之间的关系:utm-->odom
    utm_world_transform_.mult(transform_world_pose_, utm_pose_with_orientation.inverse());

    //odom-->utm坐标系转换
    utm_world_trans_inverse_ = utm_world_transform_.inverse();

    ROS_INFO_STREAM("Transform world frame pose is: " << transform_world_pose_);
    ROS_INFO_STREAM("World frame->utm transform is " << utm_world_transform_);

    transform_good_ = true;

    // Send out the (static) UTM transform in case anyone else would like to use it.
    // 发布tf,utm与world_frame_id_之间的变换.
    if (broadcast_utm_transform_)
    {
      geometry_msgs::TransformStamped utm_transform_stamped;
      utm_transform_stamped.header.stamp = ros::Time::now();
      utm_transform_stamped.header.frame_id = (broadcast_utm_transform_as_parent_frame_ ? "utm" : world_frame_id_);
      utm_transform_stamped.child_frame_id = (broadcast_utm_transform_as_parent_frame_ ? world_frame_id_ : "utm");
      utm_transform_stamped.transform = (broadcast_utm_transform_as_parent_frame_ ?
                                           tf2::toMsg(utm_world_trans_inverse_) : tf2::toMsg(utm_world_transform_));
      utm_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                         0.0 : utm_transform_stamped.transform.translation.z);
      utm_broadcaster_.sendTransform(utm_transform_stamped);
    }
  }
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

    transform_utm_pose_.setOrigin(tf2::Vector3(utm_x, utm_y, msg->altitude));
    transform_utm_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
    ROS_INFO_STREAM("has_transform_gps_ " << has_transform_gps_);
  }

//将gps数据转化为utm数据，存储于latest_utm_pose_和latest_utm_covariance_变量。
/*
* 不考虑调用datum服务的情况.
* 1. 首次调用:
*    1.1 设置使用第一次接受的数据,设置utm坐标系的位置.存放与transform_utm_pose_
*    1.2 设置has_transform_gps_为true.
* 2. 非首次调用:
*    1.1 将gps数据转化为utm数据,存储于latest_utm_pose_和latest_utm_covariance_
*/
void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  gps_frame_id_ = msg->header.frame_id;     //gps消息frame_id设置gps_frame_id_。

  if (gps_frame_id_.empty())
  {
    ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
      "origin.");
  }

  // Make sure the GPS data is usable
  bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                   !std::isnan(msg->altitude) &&
                   !std::isnan(msg->latitude) &&
                   !std::isnan(msg->longitude));

  if (good_gps)
  {
    // If we haven't computed the transform yet, then
    // store this message as the initial GPS data to use
    //如果是第一帧gps数据，且不使用手动设置datum，则将此帧数据设为utm坐标系的原点，也就是transform_utm_pose_变量。
    if (!transform_good_)
    {
      setTransformGps(msg);
      transform_good_ = true;
    }

    utm_data curr_utm;
    static utm_data last_utm;
    double yaw, roll, pitch;

    std::string utm_zone_tmp;
    LLtoUTM(msg->latitude, msg->longitude, curr_utm.utm_y, curr_utm.utm_x, utm_zone_tmp);
    latest_utm_pose_.setOrigin(tf2::Vector3(curr_utm.utm_x, curr_utm.utm_y, msg->altitude));
    latest_utm_covariance_.setZero();

    if(last_utmX_set && last_utmY_set){
        createOrientation(last_utm,curr_utm,roll,pitch,yaw);
    }

    // Copy the measurement's covariance matrix so that we can rotate it later
    for (size_t i = 0; i < POSITION_SIZE; i++)
    {
      for (size_t j = 0; j < POSITION_SIZE; j++)
      {
        latest_utm_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
      }
    }

    gps_update_time_ = msg->header.stamp;
    gps_updated_ = true;

    last_utm.utm_x = curr_utm.utm_x;
    last_utm.utm_y = curr_utm.utm_y;
    last_utmX_set = true;
    last_utmY_set = true;

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps2utm_node");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("gps", 1,gpsFixCallback);

  ros::spin();
}

