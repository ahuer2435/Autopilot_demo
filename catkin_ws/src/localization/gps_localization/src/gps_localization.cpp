#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include "gps_to_utm.h"

Eigen::MatrixXd latest_utm_covariance_;
tf2::Transform latest_utm_pose_;
tf2::Transform transform_gps2utm_pose_;
tf2::Transform transform_gps2world_pose_;
bool transform_good_ = false;
bool last_utm_ready = false;
std::string utm_zone_;
ros::Time gps_update_time_;

const int POSITION_SIZE = 3;

typedef struct utm_struct{
    double utm_x;
    double utm_y;
}utm_data;

utm_data curr_utm;
utm_data last_utm;
double yaw, roll, pitch;

std::string world_frame_id_ = "world";
std::string gps_frame_id_ = "gps";
std::string base_link_frame_id_ = "base_link";


double utm_x_orign = 0;
double utm_y_orign = 0;

ros::Publisher pub1_;

//前后两个位置的夹角.
void createOrientation(utm_data last_utm, utm_data curr_utm)
{
    yaw = atan2(curr_utm.utm_x - last_utm.utm_x, curr_utm.utm_y - last_utm.utm_y);
    roll = 0;
    pitch = 0;
}

//gps坐标系与utm坐标系的关系.存储于transform_gps2utm_pose_
void Gps2UtmTransform(const sensor_msgs::NavSatFixConstPtr& msg)
{
  double utm_x = 0;
  double utm_y = 0;
  std::string  utm_zone_;
  LLtoUTM(msg->latitude, msg->longitude, utm_y_orign, utm_x_orign, utm_zone_);

  transform_gps2utm_pose_.setOrigin(tf2::Vector3(utm_x_orign, utm_y_orign, msg->altitude));
  transform_gps2utm_pose_.setRotation(tf2::Quaternion::getIdentity());
  has_transform_utm_ = true;
  ROS_INFO_STREAM("has_transform_utm_ " << has_transform_utm_);
}

void Gps2WorldTransform(const sensor_msgs::NavSatFixConstPtr& msg)
{
  double utm_x = 0;
  double utm_y = 0;
  std::string  utm_zone_;
  LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone_);

  transform_gps2world_pose_.setOrigin(tf2::Vector3(utm_x-utm_x_orign, utm_y-utm_y_orign, msg->altitude));
  transform_gps2world_pose_.setRotation(tf2::Quaternion::getIdentity());
  has_transform_world_ = true;
  ROS_INFO_STREAM("has_transform_world_ " << has_transform_world_);
}

void computeTransform()
{
    if (!has_transform_utm_ && has_transform_world_)
    {
        //transform_utm_pose_是gps在utm坐标系下的位置,transform_utm_pose_corrected是robot在utm坐标系下的位置.
        tf2::Transform transform_utm_pose_corrected;
        getRobotOriginUtmPose(transform_utm_pose_, transform_utm_pose_corrected, ros::Time(0));

        // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
        tf2::Matrix3x3 mat(transform_orientation_);

        // Convert to RPY
        double imu_roll;
        double imu_pitch;
        double imu_yaw;
        mat.getRPY(imu_roll, imu_pitch, imu_yaw);

        imu_yaw += (magnetic_declination_ + yaw_offset_);


        tf2::Quaternion imu_quat;
        imu_quat.setRPY(0.0, 0.0, imu_yaw);

        //utm_pose_with_orientation是在utm坐标系下的位置和朝向.
        tf2::Transform utm_pose_with_orientation;
        utm_pose_with_orientation.setOrigin(transform_utm_pose_corrected.getOrigin());
        utm_pose_with_orientation.setRotation(imu_quat);

        //transform_gps2world_pose_,其保存的是world_frame_id_(world)和gps坐标系之间的关系
        //transform_gps2utm_pose_ 保存的是utm坐标系和gps坐标系之间的关系
        //utm_world_transform_保存的是world和utm坐标系之间的关系:utm-->world
        utm_world_transform_.mult(transform_gps2world_pose_, transform_gps2utm_pose_.inverse());

        //world-->utm坐标系转换
        utm_world_trans_inverse_ = utm_world_transform_.inverse();

        transform_good_ = true;

        // 发布tf,utm与world_frame_id_之间的变换.
        if (broadcast_utm_transform_)
        {
            geometry_msgs::TransformStamped utm_transform_stamped;
            utm_transform_stamped.header.stamp = ros::Time::now();
            utm_transform_stamped.header.frame_id = world_frame_id_;
            utm_transform_stamped.child_frame_id =  "utm";
            utm_transform_stamped.transform = tf2::toMsg(utm_world_transform_);
            utm_transform_stamped.transform.translation.z = 0.0;
            utm_broadcaster_.sendTransform(utm_transform_stamped);
        }
    }
}

bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans)
{
  bool retVal = true;

  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform,
                 targetFrameTrans);
  }
  catch (tf2::TransformException &ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    try
    {
      tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0)).transform,
                   targetFrameTrans);

      ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                    " was unavailable for the time requested. Using latest instead.\n");
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                    " to " << targetFrame << ". Error was " << ex.what() << "\n");

      retVal = false;
    }
  }

  // Transforming from a frame id to itself can fail when the tf tree isn't
  // being broadcast (e.g., for some bag files). This is the only failure that
  // would throw an exception, so check for this situation before giving up.
  if (!retVal)
  {
    if (targetFrame == sourceFrame)
    {
      targetFrameTrans.setIdentity();
      retVal = true;
    }
  }

  return retVal;
}

bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         tf2::Transform &targetFrameTrans)
{
  return lookupTransformSafe(buffer, targetFrame, sourceFrame, time, ros::Duration(0), targetFrameTrans);
}

void getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,tf2::Transform &robot_odom_pose,const ros::Time &transform_time)
{
    tf2_ros::Buffer tf_buffer_;
  robot_odom_pose.setIdentity();

  // Remove the offset from base_link
  tf2::Transform gps_offset_rotated;

  bool can_transform = lookupTransformSafe(tf_buffer_,base_link_frame_id_,
                                           gps_frame_id_, transform_time,
                                           transform_timeout_, gps_offset_rotated);

  if (can_transform)
  {
    tf2::Transform robot_orientation;
    can_transform = lookupTransformSafe(tf_buffer_,world_frame_id_,
                                        base_link_frame_id_,transform_time,
                                        transform_timeout_,robot_orientation);

    if (can_transform)
    {
      // Zero out rotation because we don't care about the orientation of the
      // GPS receiver relative to base_link
      gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
      gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
      robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << world_frame_id_ << "->" << base_link_frame_id_ <<
        " transform. Will not remove offset of navsat device from robot's origin.");
    }
  }
  else
  {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<" transform. Will not remove offset of navsat device from robot's origin.");
  }
}

bool prepareGpsOdometry(nav_msgs::Odometry &gps_odom)
{
  bool new_data = false;

  if (transform_good_ && gps_updated_ && odom_updated_)
  {
    tf2::Transform transformed_utm_gps;

    //utm_world_transform_保存utm-->wrold坐标系变换，latest_utm_pose_代表gps接收器在utm坐标系下的坐标，相乘可得gps接收器
    //在world坐标系下坐标。
    transformed_utm_gps.mult(utm_world_transform_, latest_utm_pose_);
    //因为latest_utm_pose_中只有位置，没有朝向，所以设置朝向
    transformed_utm_gps.setRotation(tf2::Quaternion::getIdentity());

    // Set header information stamp because we would like to know the robot's position at that timestamp
    //这里的world_frame_id_应该是odom，里程计坐标系
    gps_odom.header.frame_id = world_frame_id_;
    gps_odom.header.stamp = gps_update_time_;

    // Want the pose of the vehicle origin, not the GPS
    //根据gps在odom下的位置和此位置的时间戳，获取机器人在odom下的位置：transformed_utm_robot。
    tf2::Transform transformed_utm_robot;
    getRobotOriginWorldPose(transformed_utm_gps, transformed_utm_robot, gps_odom.header.stamp);

    // Rotate the covariance as well
    //将旋转矩阵拓展为6x6矩阵，然后对协方差进行旋转。
    tf2::Matrix3x3 rot(utm_world_transform_.getRotation());
    Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
    rot_6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
    {
      rot_6d(rInd, 0) = rot.getRow(rInd).getX();
      rot_6d(rInd, 1) = rot.getRow(rInd).getY();
      rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Rotate the covariance
    latest_utm_covariance_ = rot_6d * latest_utm_covariance_.eval() * rot_6d.transpose();

    // Now fill out the message. Set the orientation to the identity.
    //从transformed_utm_robot提取位姿信息
    tf2::toMsg(transformed_utm_robot, gps_odom.pose.pose);
    gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

    // Copy the measurement's covariance matrix so that we can rotate it later
    //跟新gps_odom中的协方差矩阵。
    for (size_t i = 0; i < POSE_SIZE; i++)
    {
      for (size_t j = 0; j < POSE_SIZE; j++)
      {
        gps_odom.pose.covariance[POSE_SIZE * i + j] = latest_utm_covariance_(i, j);
      }
    }

    // Mark this GPS as used
    gps_updated_ = false;
    new_data = true;
  }

  return new_data;
}


void publishPoseStamped(const sensor_msgs::NavSatFixConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = world_frame_id_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = curr_utm.utm_y - utm_y_orign;
    pose.pose.position.y = curr_utm.utm_x - utm_x_orign;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    pub1_.publish(pose);
}

//发布world_frame_id_和gps_frame_id_的转化tf关系.
void publishTF()
{
    static tf2_ros::TransformBroadcaster br_;           //如果没有声明为static,就不会出现/tf话题.重点mark下,这个问题,调试了蛮久
    geometry_msgs::TransformStamped transformStamped;   //且如果没有实际数据进来,也不会出现/tf话题.
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = world_frame_id_;
    transformStamped.child_frame_id = gps_frame_id_;

    transformStamped.transform.translation.x = curr_utm.utm_x - utm_x_orign;
    transformStamped.transform.translation.y = curr_utm.utm_y - utm_y_orign;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    br_.sendTransform(transformStamped);
}

//将gps 位姿数据转化为utm位姿数据，gps位姿数据设置为utm坐标系的原点和朝向
void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
{
    LLtoUTM(msg->latitude, msg->longitude, utm_y_orign, utm_x_orign, utm_zone_);
    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " << msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utm_x_orign << ", " << utm_y_orign << ")");
}


void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    // Make sure the GPS data is usable
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&  !std::isnan(msg->latitude) && !std::isnan(msg->longitude));

    if (good_gps)
    {
        if (!has_transform_utm_) {
            Gps2UtmTransform(msg);
        }

        if (!has_transform_world_) {
            Gps2WorldTransform(msg);
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

        if(last_utm_ready == true){
            createOrientation(last_utm,curr_utm);
        }

        publishTF();
        publishPoseStamped();

        last_utm.utm_x = curr_utm.utm_x;
        last_utm.utm_y = curr_utm.utm_y;
        last_utm_ready = true;
        gps_update_time_ = msg->header.stamp;
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
