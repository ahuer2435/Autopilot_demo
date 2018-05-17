#include <ros/ros.h>
#include "print.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    print_log();

    ros::spin();

    return 0;
}


