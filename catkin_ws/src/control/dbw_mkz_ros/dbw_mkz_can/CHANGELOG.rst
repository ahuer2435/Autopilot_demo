^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_mkz_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.12 (2018-01-30)
-------------------
* Updated firmware versions
* Moved ModuleVersion class and look-up-tables to exported header files (for use by other packages)
* Added power fault bit to report when modules lose power
* Added missing warning about steering fault preventing enable
* Added roslaunch argument to set use_sim_time or not
* Added roslaunch tests
* Only warn once for each unknown module version
* Contributors: Kevin Hallenbeck

1.0.11 (2017-10-19)
-------------------
* Updated firmware versions
* Added missing dependencies
* Contributors: Kevin Hallenbeck

1.0.10 (2017-10-03)
-------------------
* Updated steering firmware version
* Renamed feature name
* Contributors: Kevin Hallenbeck

1.0.9 (2017-09-19)
------------------
* Added warning to update old firmware
* Added link to request a license
* Added more detail to fault warnings
* Contributors: Kevin Hallenbeck

1.0.8 (2017-09-07)
------------------
* Migrated from dataspeed_can_msgs to can_msgs
* Contributors: Kevin Hallenbeck

1.0.7 (2017-08-21)
------------------
* Removed steering report driver activity bit
* Replaced connector fault with timeout, and warn on timeout
* Keep track of module firmware versions
* Added gear rejection enumeration to gear report
* Added licensing and VIN
* Added wheel positions report (replaces suspension report)
* Added option to use buttons for enable/disable, or not
* Added enable button combination for Mondeo without ACC (set_dec and cc_res)
* Added steering wheel left D-Pad buttons
* Updated ackermann steering parameters (including steering ratio)
* Prioritize the local include folder (there were issues with catkin workspace overlays)
* Fixed accel orientation to match the ROS standard
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.6 (2017-06-21)
------------------
* Added frame_id parameter for IMU and Twist messages
* Properly handle IMU unknown covariance and fields that are not present
* Removed SuspensionReport (data was unintelligible)
* Reorganized launch files.
* Swapped lateral and longitudinal acceleration in IMU message.
* Export dispatch.h for use by other packages
* Added clear bit to command messages
* Updated nodelet to the PLUGINLIB_EXPORT_CLASS macro
* Additional dependencies
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.5 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Unique target names
* Contributors: Kevin Hallenbeck

1.0.4 (2016-12-06)
------------------
* Added brake and throttle thrashing scripts to try and induce faults
* Changed wheel speeds to signed values
* Contributors: Kevin Hallenbeck, Joshua Whitley

1.0.3 (2016-11-17)
------------------
* Added QUIET bit to disable driver override audible warning
* Print brake/throttle/steering firmware versions
* Handle and report steering faults (FLTBUS1 and FLTBUS2)
* Contributors: Kevin Hallenbeck

1.0.2 (2016-11-07)
------------------
* Configurable steering ratio
* Contributors: Kevin Hallenbeck

1.0.1 (2016-10-10)
------------------
* Added support for apt-get binary packages
* Added twist message computed from vehicle speed and steering wheel angle.
* Contributors: Kevin Hallenbeck

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Micho Radovnikovich
