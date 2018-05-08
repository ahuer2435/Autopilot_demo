#! /bin/bash
roscore&
sleep 3s
roslaunch nmea_navsat_driver gps_driver.launch& 
sleep 2s
rosrun navsat_localization navsat_localization&
rosrun msg_convert pose_heading_vel_node&
rosrun rtk_planning rtk_record_node&

# rosbag play data/gps_20180328 --loop

while [ true ]; do
	sleep 30s
done
