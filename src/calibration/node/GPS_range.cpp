#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>

namespace sampling {
class GPSRange {
 public:
  GPSRange(ros::NodeHandle &nh, const ros::NodeHandle &rh) {
    if (!rh.getParam("file_name", file_name_)) {
      ROS_INFO_STREAM("Error! Missing file name!");
    }

    std::string file_dir = ros::package::getPath("calibration") + "/data/";
    file_name_ = file_dir + file_name_;
    gps_location_sub1_ =
        nh.subscribe("GPS_channel1", 1, &GPSRange::RecordGPSCallback, this);
    gps_location_sub2_ =
        nh.subscribe("GPS_channel2", 1, &GPSRange::RecordGPSCallback, this);
  }

 private:
  std::string file_name_;

  boost::optional<double> latitude_min_, latitude_max_, longitude_min_,
      longitude_max_;

  void RecordGPSCallback(const sensor_msgs::NavSatFix &msg) {
    if (latitude_min_.is_initialized()) {
      latitude_min_ =
          boost::optional<double>{std::min(msg.latitude, latitude_min_.get())};
    } else {
      latitude_min_ = boost::optional<double>{msg.latitude};
    }

    if (longitude_min_.is_initialized()) {
      longitude_min_ = boost::optional<double>{
          std::min(msg.longitude, longitude_min_.get())};
    } else {
      longitude_min_ = boost::optional<double>{msg.longitude};
    }

    if (latitude_max_.is_initialized()) {
      latitude_max_ =
          boost::optional<double>{std::max(msg.latitude, latitude_max_.get())};
    } else {
      latitude_max_ = boost::optional<double>{msg.latitude};
    }

    if (longitude_max_.is_initialized()) {
      longitude_max_ = boost::optional<double>{
          std::max(msg.longitude, longitude_max_.get())};
    } else {
      longitude_max_ = boost::optional<double>{msg.longitude};
    }

    if (latitude_min_.is_initialized() && longitude_min_.is_initialized() &&
        latitude_max_.is_initialized() && longitude_max_.is_initialized()) {
      std::ofstream GPS_range;
      GPS_range.open(file_name_);
      GPS_range << std::setprecision(10) << latitude_min_.get() << ","
                << std::setprecision(10) << latitude_max_.get() << "\n";

      GPS_range << std::setprecision(10) << longitude_min_.get() << ","
                << std::setprecision(10) << longitude_max_.get() << "\n";
      GPS_range.close();
    }
  }

  ros::Subscriber gps_location_sub1_;
  ros::Subscriber gps_location_sub2_;
};
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "GPS_range");
  ros::NodeHandle nh, rh("~");
  sampling::GPSRange node(nh, rh);
  ros::spin();
  return 0;
}