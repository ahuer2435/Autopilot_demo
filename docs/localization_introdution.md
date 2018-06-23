# localization 栈中目前包含三个包,如下:
* navsat_localization: 将gps数据转化为utm坐标数据.现在使用的数据.
* gps_localization: 将gps数据转化为odom坐标系下数据,构建utm->odom->base_link->gps tf tree.暂时用于仿真,还未合入系统.
* utm_localization: gps数据转化为utm数据,并发布tf和里程数据,暂时没有使用.
