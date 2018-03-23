#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);

  typedef sync_policies::ExactTime<Image, CameraInfo> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

#if 0

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>


void callback(const sensor_msgs::NavSatFixConstPtr &gps_input, const geometry_msgs::QuaternionStampedConstPtr &heading_input)
{
  // Solve all of perception here...
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_heading_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "gps", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> heading_sub(nh, "heading", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, heading_sub);

    //sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
#endif
