^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_can_usb
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2017-10-19)
------------------
* Added missing roslaunch dependency
* Added support for CAN error frames
* Contributors: Kevin Hallenbeck

1.0.6 (2017-09-07)
------------------
* Only warn about dropped messages when the drop count changes
* Migrated from dataspeed_can_msgs to can_msgs (this will break dependent packages)
* Contributors: Kevin Hallenbeck

1.0.5 (2017-08-21)
------------------
* Prioritize the local include folder (there were issues with catkin workspace overlays)
* Updated nodelets to the PLUGINLIB_EXPORT_CLASS macro
* Contributors: Kevin Hallenbeck

1.0.4 (2017-05-08)
------------------
* Priority fix for udev on Indigo
* Contributors: Kevin Hallenbeck

1.0.3 (2017-04-25)
------------------
* Install udev rules with dh_installudev
* Updated package.xml format to version 2
* Fixed copy paste error in example launch file
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.2 (2017-01-24)
------------------
* Throttle the 'not found' warning to every 10 seconds
* Install headers for use by other packages
* Option to specify USB device in constructor, option to specify textual name
* Contributors: Kevin Hallenbeck

1.0.1 (2016-11-17)
------------------
* Added example launch file
* Contributors: Kevin Hallenbeck

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Michael Lohrer
