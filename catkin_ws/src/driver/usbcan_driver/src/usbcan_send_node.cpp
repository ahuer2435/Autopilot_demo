#include <ros/ros.h>
#include <msg_convert/vehicle_cmd.h>
#include "controlcan.h"
#include "usbcan_interface.h"

static unsigned gDevType = 0;
static unsigned gDevIdx = 0;
static unsigned gChMask = 0;
static unsigned gBaud = 0;
static unsigned gTxType = 0;
static unsigned gTxSleep = 0;
static unsigned gTxFrames = 0;

ros::Publisher  pub_can_message;


static void vehicle_cmd_callback(const msg_convert::vehicle_cmd& vehicle_cmd_input)
{

    static unsigned int index = 0;
    VCI_CAN_OBJ can_message;
    can_message.ID = index++;
    can_message.TimeFlag = 0;
    can_message.SendType = gTxType;
    can_message.RemoteFlag = 0;
    can_message.ExternFlag = 0;
    can_message.DataLen = 5;
    can_message.Data[0] = vehicle_cmd_input.steering_enable;
    memcpy(&can_message.Data[1],&vehicle_cmd_input.steering_wheel_angle_cmd,4);
    send_usbcan(&can_message);

    can_message.ID = index++;
    can_message.Data[0] = vehicle_cmd_input.brake_enable;
    memcpy(&can_message.Data[1],&vehicle_cmd_input.brake_cmd,4);
    send_usbcan(&can_message);

    can_message.ID = index++;
    can_message.Data[0] = vehicle_cmd_input.throttle_enable;
    memcpy(&can_message.Data[1],&vehicle_cmd_input.throttle_cmd,4);
    send_usbcan(&can_message);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usbcan_send_node");
    ros::NodeHandle nh;

    gDevType = 16;
    gDevIdx = 0;
    gChMask = 1;
    gBaud = 0x1400;
    gTxType = 0;
    gTxSleep = 3;
    gTxFrames = 1000;
    init_usbcan( gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames);
    ros::Subscriber sub_vechile_cmd = nh.subscribe("vehicle_cmd", 10, vehicle_cmd_callback);

    ros::spin();

    close_usbcan();
    return 0;
}
