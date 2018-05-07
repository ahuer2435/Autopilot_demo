#include <ros/ros.h>
#include "pid.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle nh;

    PID pid;
    double cte = 0.1,steer_value;
    pid.Init(0.3345, 0.0011011, 2.662); //your init parameters

    int i;
    //for (in your control loop) {
    for (i = 0; i<1; i++) {
        pid.UpdateError(cte);
        steer_value = pid.TotalError();
    }
    
    ros::spin();
    return 0;
}







