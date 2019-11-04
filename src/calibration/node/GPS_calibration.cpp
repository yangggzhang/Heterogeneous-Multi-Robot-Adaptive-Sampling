#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

namespace sampling {
using GPS_SYNC = message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
                                                    sensor_msgs::NavSatFix>>;

class GPSCalibrationNode {
 public:
  GPSCalibrationNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    std::string pelican_gps_channel, rtk_gps_channel;
    if (!rh_.getParam("Pelican_GPS_channel", pelican_gps_channel)) {
      ROS_INFO_STREAM("Error! Missing Pelican GPS channel!");
    }

    if (!rh_.getParam("RTK_GPS_channel", rtk_gps_channel)) {
      ROS_INFO_STREAM("Error! Missing RTK GPS channel!");
    }

    if (!rh_.getParam("calibration_result_file", calibration_result_file_)) {
      ROS_INFO_STREAM("Error! Missing Calibration result file!");
    }

    rtk_gps_ = pelican_gps_ = Eigen::MatrixXf::Zero(0, 2);

    Pelican_GPS_sub_.reset(
        new message_filters::Subscriber<sensor_msgs::NavSatFix>(
            nh_, pelican_gps_channel, 1));

    RTK_GPS_sub_.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(
        nh_, rtk_gps_channel, 1));

    GPS_synchronizer_.reset(
        new GPS_SYNC(GPS_SYNC(60), *RTK_GPS_sub_, *Pelican_GPS_sub_));

    GPS_synchronizer_->registerCallback(boost::bind(
        &GPSCalibrationNode::GPSSynchronizerCallback, this, _1, _2));
  }

  void GPSSynchronizerCallback(
      const sensor_msgs::NavSatFixConstPtr &rtk_gps_msg,
      const sensor_msgs::NavSatFixConstPtr &pelican_gps_msg) {
    rtk_gps_.conservativeResize(rtk_gps_.rows() + 1, rtk_gps_.cols());
    rtk_gps_(rtk_gps_.rows(), 0) = rtk_gps_msg->latitude;
    rtk_gps_(rtk_gps_.rows(), 1) = rtk_gps_msg->longitude;
    pelican_gps_.conservativeResize(pelican_gps_.rows() + 1,
                                    pelican_gps_.cols());
    pelican_gps_(rtk_gps_.rows(), 0) = pelican_gps_msg->latitude;
    pelican_gps_(rtk_gps_.rows(), 1) = pelican_gps_msg->longitude;
    Eigen::MatrixXf transform =
        rtk_gps_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
            .solve(pelican_gps_);
    std::ofstream calibration_result;
    calibration_result.open(calibration_result_file_);
    calibration_result << transform;
    calibration_result.close();
  }

 private:
  ros::NodeHandle nh_, rh_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>>
      Pelican_GPS_sub_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>>
      RTK_GPS_sub_;

  std::unique_ptr<GPS_SYNC> GPS_synchronizer_;

  Eigen::MatrixXf rtk_gps_, pelican_gps_;

  std::string calibration_result_file_;
};
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "GPS_calibration");
  ros::NodeHandle nh, rh("~");
  sampling::GPSCalibrationNode node(nh, rh);
  ros::spin();
  return 0;
}