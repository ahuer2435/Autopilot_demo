/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "pure_pursuit_core.h"

namespace waypoint_follower
{

void PurePursuit::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose;
  pose_set_ = true;
}//processing frequency

void PurePursuit::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  velocity_set_ = true;
}

void PurePursuit::callbackFromWayPoints(const styx_msgs::LaneConstPtr &msg)
{
  current_waypoints_.setPath(*msg);
  waypoint_set_ = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

//根据路点index，返回相应的线速度。
//为什么参数都是0？
double PurePursuit::getCmdVelocity(int waypoint) const
{
  if (current_waypoints_.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_waypoints_.getWaypointVelocityMPS(waypoint);
  // ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

//计算前视距离lookahead_distance_，参数没有用。
//lookahead_distance_ = 当前速度（m/s）*lookahead_distance_calc_ratio_(2.0)
//最小值是6m，做大值是：当前速度（m/s）*10（相当于10s的距离）
void PurePursuit::calcLookaheadDistance(int waypoint)
{
  double current_velocity_mps = current_velocity_.twist.linear.x;
  double maximum_lookahead_distance =  current_velocity_mps * 10;
  double ld = current_velocity_mps * lookahead_distance_calc_ratio_;

  lookahead_distance_ = ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_
                      : ld > maximum_lookahead_distance ? maximum_lookahead_distance
                      : ld ;

  ROS_INFO("lookahead distance: %f",lookahead_distance_);

  return ;
}

//计算曲率：kappa
//这个计算公式貌似和标准计算公式不同。
//曲率计算公式:R=L^2/(2*x),R 为曲率半径,其倒数为对应的曲率.参考论文:一种纯追踪模型改进算法和https://zh.wikipedia.org/zh-hans/%E6%9B%B2%E7%8E%87
//L是车辆形式的圆弧的弦长,x是目标点在车辆坐标系下的横坐标.车辆坐标是以前向为y轴,右侧为x轴.圆弧的两点是车辆的当前位置和目标点的位置.
//getPlaneDistance(target, current_pose_.pose.position)表示弦长L.
//calcRelativeCoordinate(target, current_pose_.pose).y: 目标点target在车体坐标系下的横坐标,感觉应该是x,不是y
double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, current_pose_.pose.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_.pose).y;

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if(numerator > 0)
     kappa = KAPPA_MIN_;
    else
      kappa = -KAPPA_MIN_;
  }
  ROS_INFO_STREAM("kappa :" << kappa);
  return kappa;
}

// linear interpolation of next target
//根据下一个点的index，计算下一个点的pose，利用的线性插值法
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.getSize());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.getWaypointPosition(next_waypoint);
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = current_waypoints_.getWaypointPosition(next_waypoint);
  geometry_msgs::Point start = current_waypoints_.getWaypointPosition(next_waypoint - 1);

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag){
    ROS_ERROR("get Linear Equation failed.\n");
    return false;
  }


  // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
  // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise 90 degree

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = current_pose_.pose.position.x + d * unit_w1.getX();
  h1.y = current_pose_.pose.position.y + d * unit_w1.getY();
  h1.z = current_pose_.pose.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.pose.position.x + d * unit_w2.getX();
  h2.y = current_pose_.pose.position.y + d * unit_w2.getY();
  h2.z = current_pose_.pose.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = current_pose_.pose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.pose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    //displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}

//检验车是否跟随线路。
//计算当前位置与路点1和路点2的直线距离，距离小于0.2
//计算当前位置与路点1的相对角度，小于5
//则视为跟踪状态，否则不在跟踪状态
//问题：为什么是和路点1和路点2做比较，假设规划结束，当前位置与路点1最为接近。
bool PurePursuit::verifyFollowing() const
{
  double a = 0;
  double b = 0;
  double c = 0;
  //参数a，b，c确定了过两个点（路点1和路点2）的一条直线L。
  getLinearEquation(current_waypoints_.getWaypointPosition(1), current_waypoints_.getWaypointPosition(2), &a, &b, &c);
  //当前位置到直线L的距离。
  double displacement = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);
  //当前位置与过路点1的夹角。不明白两点之间的角度是怎么计算的。
  double relative_angle = getRelativeAngle(current_waypoints_.getWaypointPose(1), current_pose_.pose);
  //ROS_ERROR("side diff : %lf , angle diff : %lf",displacement,relative_angle);
  if (displacement < displacement_threshold_ && relative_angle < relative_angle_threshold_)
  {
    // ROS_INFO("Following : True");
    return true;
  }
  else
  {
    // ROS_INFO("Following : False");
    return false;
  }
}

//由下一目标点的曲率和当前速度值计算Twist。
//线速度是当前速度;角速度分情况:
//1. 若跟踪上了,使用上一时刻的角速度.
//2. 若没有跟踪上,实时计算角速度,计算公式:角速度w = 线速度v*曲率curvature.(原理是v=w*R,curvature = 1/R)
geometry_msgs::Twist PurePursuit::calcTwist(double curvature, double cmd_velocity) const
{
  // verify whether vehicle is following the path
  bool following_flag = verifyFollowing();
  static double prev_angular_velocity = 0;

  geometry_msgs::Twist twist;
  twist.linear.x = cmd_velocity;
  if (!following_flag)
  {
    //ROS_ERROR_STREAM("Not following");
    twist.angular.z = current_velocity_.twist.linear.x * curvature;
  }
  else
  {
    twist.angular.z = prev_angular_velocity;
  }

  prev_angular_velocity = twist.angular.z;
  return twist;
}

//获取下一个路点的index：num_of_next_waypoint_，利用到了lookahead_distance_。
//遍历轨迹，下一个路点：当前位置加上前视距离之后的第一个点。若当前是最后路点，下一路点设为最后路点。
//若出错设置-1；
void PurePursuit::getNextWaypoint()
{
  //获取路点数，也就是planning中的FLAGS_rtk_trajectory_forward。
  int path_size = static_cast<int>(current_waypoints_.getSize());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    num_of_next_waypoint_ = -1;
    return;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    //如果当前点是最后路点，则下一个路点设为最后路点。
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      num_of_next_waypoint_ = i;
      return;
    }

    // if there exists an effective waypoint
    //如果当前位置与轨迹中的路点大于前视距离，则将这个路点设置下个路点。
    if (getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) > lookahead_distance_)
    {
      num_of_next_waypoint_ = i;
      //ROS_ERROR_STREAM("wp = " << i << " dist = " << getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) );
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  num_of_next_waypoint_ = -1;
  return;
}

geometry_msgs::TwistStamped PurePursuit::outputZero() const
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 0;
  twist.twist.angular.z = 0;
  twist.header.stamp = ros::Time::now();
  return twist;
}

//不明白这里对线速度的处理.
geometry_msgs::TwistStamped PurePursuit::outputTwist(geometry_msgs::Twist t) const
{
  double g_lateral_accel_limit = 5.0;
  double ERROR = 1e-8;

  geometry_msgs::TwistStamped twist;
  twist.twist = t;
  twist.header.stamp = ros::Time::now();    //这里使用当前时间作为消息时间。

  double v = t.linear.x;
  double omega = t.angular.z;

  //只有纵向速度，直接返回纵向速度。
  if(fabs(omega) < ERROR){

    return twist;
  }

  //有转速度，如果转速和线速之积超过常系数g_lateral_accel_limit，则线速设置g_lateral_accel_limit除转速，
  //线速保持不变。这里不明白？？？
  double max_v = fabs(g_lateral_accel_limit / omega);  //参考:https://github.com/udacity/CarND-Capstone/issues/113


  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);

  twist.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  twist.twist.angular.z = omega;

  return twist;
}

//由pose_set_，velocity_set_和waypoint_set_计算出cmd_vel（ros结构）。
geometry_msgs::TwistStamped PurePursuit::go()
{
  //检查当前位置，当前速度，路点数据是否准备好。
  if(!pose_set_ || !waypoint_set_ || !velocity_set_){
    if(!pose_set_) {
       ROS_WARN("position is missing");
     }
     if(!waypoint_set_) {
       ROS_WARN("waypoint is missing");
     }
     if(!velocity_set_) {
       ROS_WARN("velocity is missing");
    }
    return outputZero();
  }
  else{   //add by yanqiao。如果不加，只有有一次，这里就会永远成立，不和逻辑。
      pose_set_ = false;
      waypoint_set_ = false;
      velocity_set_ = false;
  }

  bool interpolate_flag = false;

  //设置了lookahead_distance_。
  calcLookaheadDistance(1);
  // search next waypoint
  //设置了num_of_next_waypoint_
  getNextWaypoint();
  if (num_of_next_waypoint_ == -1)
  {
    ROS_WARN("lost next waypoint");
    return outputZero();
  }
  //ROS_ERROR_STREAM("next waypoint = " <<  num_of_next_waypoint_);

  // if g_linear_interpolate_mode is false or next waypoint is first or last
  if (!linear_interpolate_ || num_of_next_waypoint_ == 0 ||
      num_of_next_waypoint_ == (static_cast<int>(current_waypoints_.getSize() - 1)))
  {
    position_of_next_target_ = current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
    return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(0)));
  }

  // linear interpolation and calculate angular velocity
  //不太明白线性插值的效果,由下一个目标点的id计算下一个目标点的位置,使用线性插值,就不是直接由id获取相应的位置了.
  interpolate_flag = interpolateNextTarget(num_of_next_waypoint_, &position_of_next_target_);

  if (!interpolate_flag)
  {
    ROS_ERROR_STREAM("lost target! ");
    return outputZero();
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);
  //至此
  return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(0)));

// ROS_INFO("linear : %lf, angular : %lf",twist.twist.linear.x,twist.twist.angular.z);

#ifdef LOG
  std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
  ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "
      << _current_waypoints.getWaypointPosition(next_waypoint).y << " " << next_target.x << " " << next_target.y
      << std::endl;
#endif
}
}
