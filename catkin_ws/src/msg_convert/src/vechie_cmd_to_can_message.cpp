#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>

#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>

#include <msg_convert/vehicle_cmd.h>


ros::Publisher  pub_vehicle_cmd;

typedef boost::shared_ptr< ::dbw_mkz_msgs::SteeringCmd const> SteeringStampedConstPtr;
typedef boost::shared_ptr< ::dbw_mkz_msgs::ThrottleCmd const> ThrottleStampedConstPtr;
typedef boost::shared_ptr< ::dbw_mkz_msgs::BrakeCmd const> BrakeStampedConstPtr;

void callback(const SteeringStampedConstPtr &steering_input, const ThrottleStampedConstPtr &throttle_input,const BrakeStampedConstPtr &brake_input)
{

   msg_convert::vehicle_cmd vehicle_cmd_msg;
   vehicle_cmd_msg.header.stamp = steering_input->header.stamp;
   vehicle_cmd_msg.steering_enable = steering_input->enable;
   vehicle_cmd_msg.steering_wheel_angle_cmd = steering_input->steering_wheel_angle_cmd;
   vehicle_cmd_msg.steering_wheel_angle_velocity = steering_input->steering_wheel_angle_velocity;

   vehicle_cmd_msg.throttle_enable = throttle_input->enable;
   vehicle_cmd_msg.throttle_cmd = throttle_input->pedal_cmd;

   vehicle_cmd_msg.brake_enable = brake_input->enable;
   vehicle_cmd_msg.brake_cmd = brake_input->pedal_cmd;

   pub_vehicle_cmd.publish(vehicle_cmd_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vechie_cmd_to_can_message_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<dbw_mkz_msgs::SteeringCmd> steering_sub(nh, "/vehicle/steering_cmd", 1);
    message_filters::Subscriber<dbw_mkz_msgs::ThrottleCmd> throttle_sub(nh, "/vehicle/throttle_cmd", 1);
    message_filters::Subscriber<dbw_mkz_msgs::BrakeCmd> brake_sub(nh, "/vehicle/brake_cmd", 1);

    typedef message_filters::sync_policies::ExactTime<dbw_mkz_msgs::SteeringCmd, dbw_mkz_msgs::ThrottleCmd, dbw_mkz_msgs::BrakeCmd> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), steering_sub, throttle_sub, brake_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    pub_vehicle_cmd = nh.advertise< msg_convert::vehicle_cmd >("vehicle_cmd", 2, true);
    ros::spin();

    return 0;
}
