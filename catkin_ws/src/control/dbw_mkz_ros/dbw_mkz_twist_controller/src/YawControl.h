/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*********************************************************************
* Yaw控制器:
* 功能:
  ** 根据实际线速度或者期望线速度和期望角速度控制方向盘转向角度.
* 参数:
  ** lateral_accel_max_: 横向加速最大值.
  ** steering_wheel_angle_max_: 最大转向角度.
  ** steering_ratio_: 转向比.
  ** wheelbase_: 轴距.
  ** cmd_vx: 期望线速度.
  ** cmd_wz: 期望角速度.
  ** speed:　实际线速度．
　　** steering_wheel_angle: 转向角．
* 说明：
　　** 转向角的计算来源于自行车运动学模型
  ** 与文档不一致的地方是转向角的计算参考了期望速度,在实际速度小于0.5,且期望速度大于0.1时.
**********************************************************************/

#ifndef YAWCONTROL_H
#define YAWCONTROL_H

#include <math.h>

namespace dbw_mkz_twist_controller {

class YawControl {
public:
  YawControl() : lateral_accel_max_(INFINITY), steering_wheel_angle_max_(8.2), steering_ratio_(1.0), wheelbase_(1.0) {}
  YawControl(double wheelbase, double steering_ratio, double steering_wheel_angle_max = INFINITY, double lateral_accel_max = INFINITY) :
    lateral_accel_max_(fabs(lateral_accel_max)), steering_wheel_angle_max_(steering_wheel_angle_max), steering_ratio_(steering_ratio), wheelbase_(wheelbase) {}
  void setWheelBase(double val) { wheelbase_ = val; }   //轴距:纵向轮距
  void setSteeringRatio(double val) { steering_ratio_ = val; }  //转向比: 方向盘转动角度与车轮相应转动的角度之比
  void setLateralAccelMax(double val) { lateral_accel_max_ = fabs(val); }   //最大侧向加速度
  double getSteeringWheelAngle(double cmd_vx, double cmd_wz, double speed)
  {
    double steering_wheel_angle;
    if (fabsf(speed) > 0.5)
    { // When moving, use measured speed to compute steering angle to improve accuracy
      steering_wheel_angle = steering_ratio_ * atan(wheelbase_ * cmd_wz / speed);
    }
    else
    { // Use commanded speed to control radius at measured speeds below 0.5 m/s
      if (fabsf(cmd_vx) > 0.1)
      {
        steering_wheel_angle = steering_ratio_ * atan(wheelbase_ * cmd_wz / cmd_vx);
      }
      else
      {
        // Hits here if both measured speed and commanded speed are small;
        // set steering to zero to avoid large changes in steering command
        // due to dividing by small numbers.
        steering_wheel_angle = 0;
      }
    }

    if (steering_wheel_angle > steering_wheel_angle_max_) {
      steering_wheel_angle = steering_wheel_angle_max_;
    } else if (steering_wheel_angle < -steering_wheel_angle_max_) {
      steering_wheel_angle = -steering_wheel_angle_max_;
    }

    return steering_wheel_angle;
  }
private:
  double lateral_accel_max_;
  double steering_wheel_angle_max_;
  double steering_ratio_;
  double wheelbase_;
};

}

#endif // YAWCONTROL_H

