#ifndef USBCAN_INTERFACE_H
#define USBCAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
int init_usbcan(unsigned DevType,unsigned DevIdx,unsigned ChMask,unsigned Band,unsigned TxType,unsigned TxSleep,unsigned TxFrame);
int send_usbcan(VCI_CAN_OBJ* can);
int close_usbcan();
#ifdef __cplusplus
}
#endif

#endif
