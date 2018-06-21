# pure_pursuit 节点
## 功能:
根据路径规划给出的轨迹,当前位置和当前速度,进行速度规划.作为后续运动控制模块的输入.
## 架构图:
![pure_pursuit](./images/pure_pursuit.png)
### 输入:
* /planning/final_waypoints  [styx_msgs/Lane]: 路径规划模块输出的轨迹.
* /msg_convert/current_pose [geometry_msgs/PoseStamped]: 定位模块输出的当前位置.
* /current_velocity [geometry_msgs/TwistStamped]: 当前速度,通常可以通过测速传感器获得,目前是gps驱动输出.

### 输出:
* /vehicle/cmd_vel_stamped [geometry_msgs/TwistStamped]: 输出规划速度.

### 消息styx_msgs/Lane:
* Header header
* Waypoint[] waypoints

### 消息styx_msgs/Waypoint:
* geometry_msgs/PoseStamped pose
* geometry_msgs/TwistStamped twist
* 轨迹包含了位置和速度信息

## 算法框架:
![waypoint_follow_arch](./images/waypoint_follow_arch.bmp)
### 计算预瞄距离lookahead_distance_:
* 输入:当前线速度,预瞄系数.
* 输出:预瞄距离.
* 当前线速度current_velocity_*系数ookahead_distance_calc_ratio_,这个系数是待调的.
* lookahead_distance_要在一个最大值和最小值之间,这个两个最值也是要调的.

### 计算下一个轨迹点index:
* 输入:轨迹,当前位置,预瞄距离.
* 输出:下一目标点的index.
* 使用当前位置current_pose_遍历轨迹,计算距离,距离大于前视距离lookahead_distance_的为下一路点index.

### 计算下一个目标位置:
* 输入:下一目标点index, 轨迹,预瞄距离.
* 输出:下一目标点的位置.
* 不使用线性差分时,直接取下一路点index对应的pose,即可.
* 使用线性差分时,pose的选取按照如下方法.
 * 选取下一路点index和下一路点的上一路点(index-1)两个点,以此构造一条直线.
 * 当前位置current_pose_,向直线做垂线d,计算垂足坐标.
 * 如果d > search_radius(lookahead_distance_),说明规划的路径不在预瞄距离内，报错.
 * 如果d = search_radius, 垂足即为下一个路点.
 * 如果d < search_radius,处理没有看明白.

### 计算曲率:
* 输入: 当前位置, 目标位置.
* 输出: 曲率
* 曲率半径公式: R = L^2/(2*x),L 车辆行驶的圆弧的弦长,圆弧的两点是车辆的当前位置和目标点的位置,x是目标点在车辆坐标系下的横坐标.
* 曲率公式: kappa = 1/R.
* 车辆坐标: 以前向为y轴,右侧为x轴,圆心为后轴中心.
* 曲率有两个最值限制,也是需要调试的.
* 计算式来历可以参考论文:一种纯追踪模型改进算法和https://zh.wikipedia.org/zh-hans/%E6%9B%B2%E7%8E%87

### 获取第一个轨迹点线速度.
* 直接使用index为0,从轨迹点中获取.

### 速度规划.
* 输入: 曲率,第一个轨迹点线速度,当前线速度.
* 输出: 规划的速度.
* 规划的线速度就是第一个轨迹点线速度.
* 规划的角速度分两种情况:
 * 处于跟踪状态,使用上一时刻的角速度.
 * 不处于跟踪状态,当前线速度与曲率相乘.
