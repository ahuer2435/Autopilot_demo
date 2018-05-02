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
* 低通滤波器:
* 功能: 将数据光滑化,即新的数据值取决于前一个时刻的值,新的采样值通过累计,可以逐渐使得新数据与采样数据保持一致.
* 参数:
  ** ready_: 低通滤波器是否可以使用,即参数a_,b_是否初始化.
  ** tau: 滤波器时间.一般远大于采样周期.进而控制a_值.这里取值0.5
  ** ts: 采样周期.这里取值0.02
  ** a_: 滤波系数.滤波系数越小,数据曲线越光滑,但灵敏度越差.
  ** b_: 1-a_
* 用法:
  ** 调用一次LowPass()和setParams()函数.初始化参数.
  ** filt()函数进行滤波;get()函数获取滤波之后的值.
* 难点:
  ** 对于参数a_的选取,一般根据需要,选取a_,这里提供了一个方法,方法总难点是滤波器时间的选取.

**********************************************************************/
#ifndef SRC_LOWPASS_H_
#define SRC_LOWPASS_H_

namespace dbw_mkz_twist_controller {

class LowPass {
public:
  LowPass() : ready_(false), last_val_(0) { a_ = 1; b_ = 0; }
  LowPass(double tau, double ts) : ready_(false), last_val_(0) { setParams(tau, ts); }
  void setParams(double tau, double ts) {
    a_ = 1 / (tau / ts + 1);
    b_ = tau / ts / (tau / ts + 1);
  }
  double get() { return last_val_; }
  double filt(double val) {
    if (ready_) {
      val = a_ * val + b_ * last_val_;
    } else {
      ready_ = true;
    }
    last_val_ = val;
    return val;
  }
private:
  bool ready_;
  double a_;
  double b_;
  double last_val_;
};

}

#endif /* SRC_LOWPASS_H_ */

