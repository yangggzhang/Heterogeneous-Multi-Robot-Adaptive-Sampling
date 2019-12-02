#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace sampling {
class GPSRecordingNode {
 public:
  GPSRecordingNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    if (!rh_.getParam("queue_size", queue_size_)) {
      ROS_INFO_STREAM("Error! Missing queue size!");
    }

    std::string gps_channel, record_gps_channel;

    if (!rh_.getParam("gps_channel", gps_channel)) {
      ROS_INFO_STREAM("Error! Missing GPS channel!");
    }
    gps_location_sub_ = nh_.subscribe(
        gps_channel, 1, &GPSRecordingNode::RecordGPSCallback, this);

    if (!rh_.getParam("record_gps_channel", record_gps_channel)) {
      ROS_INFO_STREAM("Error! Missing GPS recording service channel!");
    }

    std::string file_dir = ros::package::getPath("calibration") + "/data/";

    if (!rh_.getParam("GPS_waypoint_dir", GPS_waypoint_dir_)) {
      ROS_INFO_STREAM("Error! Missing GPS waypoints saving directory!");
    }

    GPS_waypoint_dir_ = file_dir + GPS_waypoint_dir_;

    record_gps_server_ = nh_.advertiseService(
        record_gps_channel, &GPSRecordingNode::RecordGPSService, this);
  }

 private:
  bool RecordGPSService(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res) {
    if (req.data) {
      double new_latitude_waypoint = 0.0;
      for (const double &latitude : latitude_queue_) {
        new_latitude_waypoint = new_latitude_waypoint + latitude;
      }
      new_latitude_waypoint =
          new_latitude_waypoint / (double)latitude_queue_.size();

      double new_longitude_waypoint = 0.0;
      for (const double &longitude : longitude_queue_) {
        new_longitude_waypoint = new_longitude_waypoint + longitude;
      }
      new_longitude_waypoint =
          new_longitude_waypoint / (double)longitude_queue_.size();

      latitude_waypoints_.push_back(new_latitude_waypoint);
      longitude_waypoints_.push_back(new_longitude_waypoint);

      std::ofstream GPS_waypoints;
      GPS_waypoints.open(GPS_waypoint_dir_);
      for (const double &latitude : latitude_waypoints_) {
        GPS_waypoints << std::setprecision(10) << latitude << " , ";
      }
      GPS_waypoints << '\n';
      for (const double &longitude : longitude_waypoints_) {
        GPS_waypoints << std::setprecision(10) << longitude << " , ";
      }
      GPS_waypoints << '\n';
      GPS_waypoints.close();
    }
    return true;
  }

  void RecordGPSCallback(const sensor_msgs::NavSatFix &msg) {
    if (latitude_queue_.size() >= queue_size_) {
      latitude_queue_.pop_front();
    }
    if (longitude_queue_.size() >= queue_size_) {
      longitude_queue_.pop_front();
    }
    latitude_queue_.push_back(msg.latitude);
    longitude_queue_.push_back(msg.longitude);
  }

  ros::NodeHandle nh_, rh_;

  int queue_size_;

  std::deque<double> latitude_queue_;
  std::deque<double> longitude_queue_;

  std::vector<double> latitude_waypoints_;
  std::vector<double> longitude_waypoints_;

  ros::ServiceServer record_gps_server_;
  ros::Subscriber gps_location_sub_;

  std::string GPS_waypoint_dir_;
};
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "GPS_recording");
  ros::NodeHandle nh, rh("~");
  sampling::GPSRecordingNode node(nh, rh);
  ros::spin();
  return 0;
}