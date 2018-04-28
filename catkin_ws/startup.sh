#! /bin/bash
#roscore&
sleep 3s
roslaunch nmea_navsat_driver gps_driver.launch& 
sleep 2s
rosrun navsat_localization navsat_localization&
rosrun msg_convert pose_heading_vel_node&
rosrun msg_convert pose_global_current_node&
rosrun rtk_planning rtk_replay_planner_node&
roslaunch waypoint_follower pure_pursuit.launch&
sleep 2s
rosrun mode_control mode_control_node&
roslaunch twist_controller dbw.launch&
sleep 2s
rosrun msg_convert vechie_cmd_to_can_message_node&
rosrun usbcan_driver usbcan_node&


while [ true ]; do
	sleep 30s
done
