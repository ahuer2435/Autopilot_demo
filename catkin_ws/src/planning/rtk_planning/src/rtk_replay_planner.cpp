#include<fstream>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
//#include "string_tokenizer.h"
#define FILE_NAME "./garage.csv"

using namespace std;

struct Point{
    double_t latitude;
    double_t longitude;
    double_t altitude;
    double_t time;
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
    int i = 0;

    while(getline(ss,value,',')){
        str.push_back(value);
        //cout<<str[i]<<endl;
        //i++;
    }

    Point point;

    point.latitude = std::stod(str[0]);
    point.longitude = std::stod(str[1]);
    point.altitude = std::stod(str[2]);
    point.time = std::stod(str[3]);
    cout<<4<<endl;

    cout << "point.latitude = " << point.latitude << endl;
    cout << "point.longitude = " << point.longitude << endl;
    cout << "point.altitude = " << point.altitude << endl;
    cout << "point.time = " << point.time << endl;

    complete_rtk_trajectory_.push_back(point);

  }

  file_in.close();
}


static void plan_callback(const sensor_msgs::NavSatFix& gps_input)
{

    //TODO
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_replay_planner_node");
    ros::NodeHandle nh;
    ReadTrajectoryFile(FILE_NAME);
    ros::Subscriber sub_gps = nh.subscribe("gps", 10, plan_callback);
    ros::spin();

    return 0;
}
