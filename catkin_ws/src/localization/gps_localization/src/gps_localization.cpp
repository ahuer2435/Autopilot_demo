#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "gps_to_utm.h"

#include "gps_localization/ros_filter_utilities.h"

Eigen::MatrixXd latest_utm_covariance_;
tf2::Transform latest_utm_pose_;
tf2::Transform transform_gps2utm_pose_;
tf2::Transform transform_gps2world_pose_;
tf2::Transform transform_utm_pose_;
tf2::Transform utm_world_transform_;
tf2::Transform utm_world_trans_inverse_;
ros::Duration transform_timeout_ = ros::Duration(5);
bool transform_good_ = false;
bool last_utm_ready = false;
bool gps_updated_ = false;
bool has_transform_utm_ = false;
bool has_transform_world_ = false;
std::string utm_zone_;
ros::Time gps_update_time_;

const int POSITION_SIZE = 3;
const int POSE_SIZE = 6;

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

/*
 * Gps2UtmTransform与Gps2WorldTransform都使用第一帧gps数据,计算出初始状态时,world->utm坐标系关系.
 * world坐标系就是第一帧时,车辆所处的位置.
*/
//gps坐标系与utm坐标系的关系.存储于transform_gps2utm_pose_
void Gps2UtmTransform(const sensor_msgs::NavSatFixConstPtr& msg)
{
  std::string  utm_zone_;
  LLtoUTM(msg->latitude, msg->longitude, utm_y_orign, utm_x_orign, utm_zone_);

  transform_gps2utm_pose_.setOrigin(tf2::Vector3(utm_x_orign, utm_y_orign, 0.0));
  transform_gps2utm_pose_.setRotation(tf2::Quaternion::getIdentity());
  has_transform_utm_ = true;
  ROS_INFO_STREAM("has_transform_utm_ " << has_transform_utm_);
}

//gps坐标系与world坐标系的关系.存储于transform_gps2world_pose_
//若有里程计传感器,这里应该使用里程计传感器,构建odom与gps(base_link)关系
void Gps2WorldTransform(const sensor_msgs::NavSatFixConstPtr& msg)
{
  double utm_x = 0;
  double utm_y = 0;
  std::string  utm_zone_;
  LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone_);

  transform_gps2world_pose_.setOrigin(tf2::Vector3(utm_x-utm_x_orign, utm_y-utm_y_orign, 0.0));
  transform_gps2world_pose_.setRotation(tf2::Quaternion::getIdentity());
  has_transform_world_ = true;
  ROS_INFO_STREAM("has_transform_world_ " << has_transform_world_);
}

void getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,tf2::Transform &robot_odom_pose, const ros::Time &transform_time)
{
    robot_odom_pose.setIdentity();

    // Remove the offset from base_link
    tf2::Transform gps_offset_rotated;
    tf2_ros::Buffer tf_buffer_;

    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,base_link_frame_id_,gps_frame_id_,
                                                                 transform_time,transform_timeout_,gps_offset_rotated);
    ROS_INFO("base_link->gps can_transform %d.\n", can_transform);

    if (can_transform)
    {
      tf2::Transform robot_orientation;
      can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,world_frame_id_,base_link_frame_id_,
                                                              transform_time,transform_timeout_,robot_orientation);
      ROS_INFO("world->base_link can_transform %d.\n",can_transform);

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


void publishTF()
{
    if (has_transform_utm_ && has_transform_world_)
    {
        // Convert to RPY
        double imu_roll = 0.0;
        double imu_pitch = 0.0;
        double imu_yaw = 0.0;

        tf2::Quaternion imu_quat;
        imu_quat.setRPY(imu_roll, imu_pitch, imu_yaw);

        //utm_pose_with_orientation是在utm坐标系下的位置和朝向.
        tf2::Transform utm_pose_with_orientation;
        utm_pose_with_orientation.setOrigin(transform_gps2utm_pose_.getOrigin());
        utm_pose_with_orientation.setRotation(imu_quat);

        //transform_gps2world_pose_,其保存的是gps坐标系在world坐标系下的位置(第一帧时是重合的)
        //transform_gps2utm_pose_ 保存的是gps坐标系在utm坐标系下的位置
        //utm_world_transform_保存的是utm坐标系在world坐标系下的位置
        utm_world_transform_.mult(transform_gps2world_pose_, utm_pose_with_orientation.inverse());
        //world坐标系在utm坐标系下的位置.
        utm_world_trans_inverse_ = utm_world_transform_.inverse();
        transform_good_ = true;

        // 发布tf,utm与world_frame_id_之间的变换.
        static tf2_ros::TransformBroadcaster utm_world_broadcaster_;
        geometry_msgs::TransformStamped utm_world_transform_stamped;

        static tf2_ros::TransformBroadcaster world_baselink_broadcaster_;
        geometry_msgs::TransformStamped world_baselink_transform_stamped;

        tf2::Transform transformed_world_gps;

        //utm_world_transform_保存的是utm坐标系在world坐标系下的位置，latest_utm_pose_代表gps接收器在utm坐标系下的坐标，相乘可得gps接收器
        //在world坐标系下坐标。
        transformed_world_gps.mult(utm_world_transform_, latest_utm_pose_);
        //因为latest_utm_pose_中只有位置，没有朝向，所以设置朝向
        transformed_world_gps.setRotation(tf2::Quaternion::getIdentity());

        tf2::Transform transformed_world_robot;
        getRobotOriginWorldPose(transformed_world_gps, transformed_world_robot, gps_update_time_);

        utm_world_transform_stamped.header.stamp = gps_update_time_;    //ros::Time::now();
        utm_world_transform_stamped.header.frame_id = "utm";
        utm_world_transform_stamped.child_frame_id = world_frame_id_;
        utm_world_transform_stamped.transform = tf2::toMsg(utm_world_trans_inverse_);
        utm_world_transform_stamped.transform.translation.z = 0.0;
        utm_world_broadcaster_.sendTransform(utm_world_transform_stamped);

        world_baselink_transform_stamped.header.stamp = utm_world_transform_stamped.header.stamp;
        world_baselink_transform_stamped.header.frame_id = world_frame_id_;
        world_baselink_transform_stamped.child_frame_id = base_link_frame_id_;
        world_baselink_transform_stamped.transform = tf2::toMsg(transformed_world_robot);
        world_baselink_transform_stamped.transform.translation.z = 0.0;
        world_baselink_broadcaster_.sendTransform(world_baselink_transform_stamped);
    }
}


bool prepareGpsOdometry(nav_msgs::Odometry &gps_odom)
{
  bool new_data = false;

  if (transform_good_ && gps_updated_)
  {
    tf2::Transform transformed_world_gps;

    //utm_world_transform_保存utm-->wrold坐标系变换，latest_utm_pose_代表gps接收器在utm坐标系下的坐标，相乘可得gps接收器
    //在world坐标系下坐标。
    transformed_world_gps.mult(utm_world_transform_, latest_utm_pose_);
    //因为latest_utm_pose_中只有位置，没有朝向，所以设置朝向
    transformed_world_gps.setRotation(tf2::Quaternion::getIdentity());


    // Set header information stamp because we would like to know the robot's position at that timestamp
    //这里的world_frame_id_应该是odom，里程计坐标系
    gps_odom.header.frame_id = world_frame_id_;
    gps_odom.header.stamp = gps_update_time_;
    gps_odom.child_frame_id = base_link_frame_id_;

    tf2::Transform transformed_world_robot;
    getRobotOriginWorldPose(transformed_world_gps, transformed_world_robot, gps_odom.header.stamp);


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
    tf2::toMsg(transformed_world_robot, gps_odom.pose.pose);
    gps_odom.pose.pose.position.z =  0.0;

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


void publishPoseStamped()
{
    nav_msgs::Odometry gps_odom;
    if(prepareGpsOdometry(gps_odom)){
        pub1_.publish(gps_odom);
    }
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

        latest_utm_pose_.setOrigin(tf2::Vector3(curr_utm.utm_x, curr_utm.utm_y, 0.0));
        latest_utm_covariance_.setZero();

        //这里运行出错,暂时没有找到原因.现行注掉. 找到问题:没有latest_utm_covariance_.resize(POSE_SIZE, POSE_SIZE).
        for (size_t i = 0; i < POSITION_SIZE; i++)
        {
            for (size_t j = 0; j < POSITION_SIZE; j++)
            {
                latest_utm_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
            }
        }

        if(last_utm_ready == true){
            createOrientation(last_utm,curr_utm);
        }

        gps_update_time_ = msg->header.stamp;
        gps_updated_ = true;

        publishTF();
        //publishPoseStamped();

        last_utm.utm_x = curr_utm.utm_x;
        last_utm.utm_y = curr_utm.utm_y;
        last_utm_ready = true;        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps2utm_node");
    ros::NodeHandle nh;

    latest_utm_covariance_.resize(POSE_SIZE, POSE_SIZE);
    ros::Subscriber gps_sub = nh.subscribe("gps", 1,gpsFixCallback);
    pub1_ = nh.advertise<nav_msgs::Odometry>("odom_pose", 10);
    ros::spin();
    return 0;
}
