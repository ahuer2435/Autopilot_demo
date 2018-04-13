#include <ros/ros.h>
#include <msg_convert/vehicle_cmd.h>
#include "controlcan.h"
#include "usbcan_interface.h"
#include "boost/thread.hpp"
#include "msg_convert/vehicle_status.h"

#define RX_BUFF_SIZE  1000

typedef struct {
    unsigned channel; // channel index, 0~3
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
} RX_CTX;


static unsigned gDevType = 0;
static unsigned gDevIdx = 0;
static unsigned gChMask = 0;
static unsigned gBaud = 0;
static unsigned gTxType = 0;
static unsigned gTxSleep = 0;
static unsigned gTxFrames = 0;

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

int can_data_recevie_loop()
{
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
    ros::init(argc, argv, "usbcan_recevie_node");
    ros::NodeHandle nh;

    gDevType = 16;
    gDevIdx = 0;
    gChMask = 1;
    gBaud = 0x1400;
    gTxType = 0;
    gTxSleep = 3;
    gTxFrames = 1000;
    init_usbcan( gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames);
    boost::thread reveive_loop(&CanDataReceiveLoop);
    pub_vechile_stata = nh.advertise<msg_convert::vehicle_status>("vehicle_stata", 2, true);

    ros::spin();

    close_usbcan();
    return 0;
}
