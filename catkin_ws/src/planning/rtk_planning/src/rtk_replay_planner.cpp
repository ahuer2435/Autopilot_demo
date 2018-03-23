#include<fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <msg_convert/global_pose.h>
#define FILE_NAME "./garage.csv"
#define FLAGS_rtk_trajectory_forward 80
#define FLAGS_trajectory_resolution 0.01

ros::Publisher  pub_trajectory;

using namespace std;

struct Point{
    double_t latitude;
    double_t longitude;
    double_t altitude;
    double_t time;
    double_t yaw;
};

vector<Point> complete_rtk_trajectory_;

void ReadTrajectoryFile(const std::string& filename) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open()) {
    cout << "RTKReplayPlanner cannot open trajectory file: " << filename;
    return;
  }

  std::string line,value;

  // skip the header line.
  getline(file_in, line);

  while (true) {
    getline(file_in, line);
    if (line == "") {
      break;
    }

    stringstream ss(line);
    vector<string> str;

    while(getline(ss,value,',')){
        str.push_back(value);
    }

    Point point;

    point.latitude = std::stod(str[0]);
    point.longitude = std::stod(str[1]);
    point.altitude = std::stod(str[2]);
    point.yaw = std::stod(str[3]);

    complete_rtk_trajectory_.push_back(point);

  }

  file_in.close();
}


size_t QueryPositionMatchedPoint(const Point& start_point,const vector<Point>& trajectory) {
  auto func_distance_square = [](const Point& point, const double x,const double y) {
    double dx = point.latitude - x;
    double dy = point.longitude - y;
    return dx * dx + dy * dy;
  };
  double d_min = func_distance_square(trajectory.front(), start_point.latitude,start_point.longitude);
  size_t index_min = 0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    double d_temp = func_distance_square(trajectory[i], start_point.latitude,start_point.longitude);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return index_min;
}

bool Plan(const Point& start_point,std::vector<Point>* ptr_discretized_trajectory) {

  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    cout << "RTKReplayPlanner doesn't have a recorded trajectory or "
              "the recorded trajectory doesn't have enough valid trajectory "
              "points.";
    return false;
  }

  std::size_t matched_index = QueryPositionMatchedPoint(start_point, complete_rtk_trajectory_);

  std::size_t forward_buffer = FLAGS_rtk_trajectory_forward;
  std::size_t end_index = matched_index + forward_buffer >= complete_rtk_trajectory_.size() ? complete_rtk_trajectory_.size() - 1 : matched_index + forward_buffer - 1;

  if (ptr_discretized_trajectory->size() > 0) {
    ptr_discretized_trajectory->clear();
  }

  ptr_discretized_trajectory->insert(
      ptr_discretized_trajectory->begin(),
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index + 1);

  // reset relative time
  double zero_time = complete_rtk_trajectory_[matched_index].time;;
  for (std::size_t i = 0; i < ptr_discretized_trajectory->size(); ++i) {
    (*ptr_discretized_trajectory)[i].time -= zero_time;
  }

  // check if the trajectory has enough points;
  // if not, append the last points multiple times and
  // adjust their corresponding time stamps.
  while (ptr_discretized_trajectory->size() < FLAGS_rtk_trajectory_forward) {
    const auto& last_point = ptr_discretized_trajectory->back();
    ptr_discretized_trajectory->push_back(last_point);
    ptr_discretized_trajectory->back().time += FLAGS_trajectory_resolution;
  }
  return true;
}

static double_t quad_to_yaw(const geometry_msgs::Quaternion &msg)
{
    tf::Quaternion quad;
    double_t roll, pitch, yaw;

    tf::quaternionMsgToTF(msg,quad);
    tf::Matrix3x3(quad).getRPY(roll, pitch, yaw);

    return yaw;
}

static void plan_callback(const msg_convert::global_pose& global_pose_input)
{
    
    bool flags = false;
    Point curr_pose;
    std::vector<Point> curr_discretized_trajectory;
    geometry_msgs::PoseArray curr_trajectory;
    curr_pose.latitude = global_pose_input.pose.latitude;
    curr_pose.longitude = global_pose_input.pose.longitude;
    curr_pose.altitude = global_pose_input.pose.altitude;
    curr_pose.yaw = quad_to_yaw(global_pose_input.heading.quaternion);

    flags = Plan(curr_pose,&curr_discretized_trajectory);
    if(!flags){
       cout<<"Plan failed."<<endl;
       return;
    }
    for(int i = 0; i < curr_discretized_trajectory.size();i++){
        curr_trajectory.poses[i].position.x =  curr_discretized_trajectory[i].latitude;
        curr_trajectory.poses[i].position.y =  curr_discretized_trajectory[i].longitude;
        curr_trajectory.poses[i].position.z =  curr_discretized_trajectory[i].altitude;
    }
    pub_trajectory.publish(curr_trajectory);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_replay_planner_node");
    ros::NodeHandle nh;
    ReadTrajectoryFile(FILE_NAME);
    ros::Subscriber sub_gps = nh.subscribe("global_pose", 10, plan_callback);
    pub_trajectory = nh.advertise< geometry_msgs::PoseArray >("trajectory", 2, true);
    ros::spin();

    return 0;
}
