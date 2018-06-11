# 基于datespeedinc的仿真包，实现车辆的仿真。
## 命令：
```
roslaunch dbw_mkz_can dbw.launch
roslaunch dbw_mkz_description rviz.launch
rosbag play data/mkz_20151207.new.bag --clock --loop


rosbag play data/gps_20180328.bag
rosbag play data/mkz_20151207.new.bag --clock --loop
rosrun utm_localization gps2utm_node
roslaunch nmea_navsat_driver gps_driver.launch
rosrun tf static_transform_publisher 0 0 0 0 0 1 /gps /base_footprint 1
```
