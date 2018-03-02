nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

1. nmea_topic_serial_reader 从串口读入数据，发布nmea_msgs/Sentence类型的gps数据。
2. nmea_topic_driver 订阅nmea_msgs/Sentence类型数据，转化为sensor_msgs/NavSatFix类型数据。可作为节点/navsat_transform_node的输入。
3. 启动命令：roslaunch nmea_navsat_driver gps_driver.launch 对应节点nmea_topic_driver；其余命令用rosrun启动。
