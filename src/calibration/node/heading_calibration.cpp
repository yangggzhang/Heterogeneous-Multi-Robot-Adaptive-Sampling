#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <stdlib.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace sampling {
class HeadingCalibrationNode {
 public:
  HeadingCalibrationNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    if (!rh_.getParam("magnetic_declination_radians",
                      magnetic_declination_radians_)) {
      ROS_INFO_STREAM("Error! Missing magnetic declination radians!");
    }

    if (!rh_.getParam("calibration_times", calibration_times_)) {
      ROS_INFO_STREAM("Error! Missing calibration total times!");
    }

    calibration_count_ = 0;

    std::string odom_channel, cmd_vel_channel;

    if (!rh_.getParam("odom_channel", odom_channel)) {
      ROS_INFO_STREAM("Error! Missing odometry channel!");
    }

    if (!rh_.getParam("cmd_vel_channel", cmd_vel_channel)) {
      ROS_INFO_STREAM("Error! Missing velocity command channel!");
    }

    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_channel, 1);

    calibration_started_ = false;
    is_calibration_on_going_ = false;

    odometry_sub_ = nh_.subscribe(
        odom_channel, 1, &HeadingCalibrationNode::OdometeyCallback, this);

    event_timer_ = nh_.createTimer(
        ros::Duration(0.2), &HeadingCalibrationNode::TimerCallback, this);

    start_hearding_calibration_server_ = nh_.advertiseService(
        "calibrate_heading", &HeadingCalibrationNode::StartCalibrationService,
        this);

    if (!rh_.getParam("calibration_result_dir", calibration_result_dir_)) {
      ROS_INFO_STREAM("Error! Missing calibration results saving directory!");
    }
  }

 private:
  ros::NodeHandle nh_, rh_;

  double magnetic_declination_radians_;

  bool is_calibration_on_going_, calibration_started_;

  std::string calibration_result_dir_;

  ros::Subscriber odometry_sub_;

  ros::Publisher velocity_pub_;

  ros::Timer event_timer_;

  ros::ServiceServer start_hearding_calibration_server_;

  std::vector<double> heading_errors_;

  int calibration_times_;

  int calibration_count_;

  void OdometeyCallback(const nav_msgs::Odometry &msg) {
    double y_pos = msg.pose.pose.position.y;
    double x_pos = msg.pose.pose.position.x;
    if (is_calibration_on_going_) {
      heading_errors_.push_back(atan2(y_pos, x_pos));
    }
  }

  void TimerCallback(const ros::TimerEvent &) {
    if (calibration_started_) {
      if (calibration_count_++ < calibration_times_) {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;
        velocity_pub_.publish(msg);
        ros::spinOnce();
        is_calibration_on_going_ = true;
      } else {
        calibration_count_ = 0;
        calibration_started_ = false;
        is_calibration_on_going_ = false;

        double heading_error = 0.0;

        for (const double &error : heading_errors_) {
          heading_error += error;
        }

        heading_error = heading_error / (double)heading_errors_.size();
        heading_errors_.clear();

        ROS_INFO_STREAM(
            "Original declination : " << magnetic_declination_radians_);
        ROS_INFO_STREAM("Offset : " << heading_error);
        double new_magnetic_declination =
            heading_error + magnetic_declination_radians_;
        ROS_INFO_STREAM("Heading Error : " << new_magnetic_declination);
        std::ofstream heading_error_file;
        heading_error_file.open(calibration_result_dir_);
        heading_error_file << heading_error;
        heading_error_file.close();
      }
    } else {
      calibration_count_ = 0;
      calibration_started_ = false;
      is_calibration_on_going_ = false;
      heading_errors_.clear();
    }
  }

  bool StartCalibrationService(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
    if (req.data) {
      calibration_count_ = 0;
      heading_errors_.clear();
      calibration_started_ = true;
    }
    return true;
  }
};
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "heading_calibration");
  ros::NodeHandle nh, rh("~");
  sampling::HeadingCalibrationNode node(nh, rh);
  ros::spin();
  return 0;
}