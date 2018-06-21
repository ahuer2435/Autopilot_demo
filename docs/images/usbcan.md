# 关于使用usbcan驱动权限问题：
## 问题描述：
zlg usbcan-I-mini 驱动是以共享库像是提供的，上层应用可以通过调用相关接口访问can。但是其驱动实现时是直接操作/dev/bus/usb/aaa/bbb，aaa是usb 的id，bbb是该usb上的设备id，
具体可以通过lsusb获取其值。
通过strace可以跟踪程序调用过程，定位是哪里出现的权限问题，定位具体的设备文件。

## 解决方法
默认情况下，只有管理员才能访问，所以在打开设备时会出错。这种情况可以有两种方式解决：
* 直接将修改设备文件权限6777.
* 通过udev实现权限修改，可以参考：
[http://www.cnblogs.com/linuxprobe/p/5377851.html](http://www.cnblogs.com/linuxprobe/p/5377851.html)
[https://www.ibm.com/developerworks/cn/linux/l-cn-udev/index.html?ca=drs-cn-0304](https://www.ibm.com/developerworks/cn/linux/l-cn-udev/index.html?ca=drs-cn-0304)
