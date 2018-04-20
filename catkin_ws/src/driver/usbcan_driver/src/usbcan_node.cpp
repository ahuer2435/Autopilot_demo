#include <ros/ros.h>
#include <msg_convert/vehicle_cmd.h>
#include "controlcan.h"
#include "usbcan_interface.h"
#include "boost/thread.hpp"
#include "msg_convert/vehicle_status.h"

static unsigned gDevType = 0;
static unsigned gDevIdx = 0;
static unsigned gChMask = 0;
static unsigned gBaud = 0;
static unsigned gTxType = 0;
static unsigned gTxSleep = 0;
static unsigned gTxFrames = 0;

#define RX_BUFF_SIZE  1000

typedef struct {
    unsigned channel; // channel index, 0~3
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
} RX_CTX;

ros::Publisher  pub_can_message;
ros::Publisher pub_vechile_stata;

int verify_frame(VCI_CAN_OBJ *can)
{
#if 0
    if (can->DataLen > 8) return -1; // error: data length
    unsigned bcc = 0;
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
        bcc ^= can->Data[i];
    if ((can->ID & 0xff) != bcc) return -1; // error: data checksum
    if (((can->ID >> 8) & 7) != (can->DataLen - 1)) return -1; // error: data length
    if (!can->ExternFlag) return 0; // std-frame ok
    if (((can->ID >> 11) & 0x7ff) != (can->ID & 0x7ff)) return -1; // error: frame id
    if (((can->ID >> 22) & 0x7f) != (can->ID & 0x7f)) return -1; // error: frame id
#endif
    return 0; // ext-frame ok
}

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

int can_data_recevie_loop()
{
    msg_convert::vehicle_status vehicle_status;
    VCI_CAN_OBJ can[RX_BUFF_SIZE];
    memset(can, 0, sizeof(VCI_CAN_OBJ)*RX_BUFF_SIZE);
    int i,frame_size;
    int channel = 0;
    while (1)
    {
        frame_size = receive_usbcan(can,channel);
        if(frame_size == -1){
            printf("error, frame_size == -1\n");
            return -1;
        }

        for(i = 0; i < frame_size; frame_size++){
            if (verify_frame(&can[i]) == -1){
                printf("verify_frame fail.\n");
                continue;
            }
            vehicle_status.id = can[i].ID;
            vehicle_status.datalen = can[i].DataLen;
            memcpy(&vehicle_status.data,&can[i].Data,can[i].DataLen);
            pub_vechile_stata.publish(vehicle_status);
        }
    }
    return 0;
}


void CanDataReceiveLoop(void)
{
    can_data_recevie_loop();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usbcan_node");
    ros::NodeHandle nh;

    gDevType = 16;
    gDevIdx = 0;
    gChMask = 1;
    gBaud = 0x1400;
    gTxType = 2;
    gTxSleep = 3;
    gTxFrames = 1000;
    init_usbcan( gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames);

    pub_vechile_stata = nh.advertise<msg_convert::vehicle_status>("vehicle_stata", 2, true);

    ros::Subscriber sub_vechile_cmd = nh.subscribe("vehicle_cmd", 10, vehicle_cmd_callback);
    boost::thread reveive_loop(&CanDataReceiveLoop);

    ros::spin();

    close_usbcan();
    return 0;
}
