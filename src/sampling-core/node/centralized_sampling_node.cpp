#include <ros/ros.h>
#include <string>
#include "sampling-core/gmm_utils.h"
#include "sampling-core/sampling_visualization.h"
#include "sampling-core/utils.h"

namespace sampling {
class CentralizedSamplingNode {
 public:
  CentralizedSamplingNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    load_parameter();
  }

  bool load_parameter() {
    std::string ground_truth_location_path, ground_truth_temperature_path;

    if (!rh_.getParam("ground_truth_location_path",
                      ground_truth_location_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth location data!");
      return false;
    }

    if (!rh_.getParam("ground_truth_temperature_path",
                      ground_truth_temperature_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth temperature data!");
      return false;
    }

    if (!load_ground_truth_data(
            ground_truth_location_path, ground_truth_temperature_path,
            ground_truth_location_, ground_truth_temperature_)) {
      ROS_INFO_STREAM("Error! Can not load ground truth data!");
      return false;
    }

    /// toda load ground truth data

    if (!rh_.getParam("convergence_threshold", convergence_threshold_)) {
      ROS_INFO_STREAM("Error! Missing gaussian process convergence threshold!");
      return false;
    }

    if (!rh_.getParam("ugv_max_speed", ugv_max_speed_)) {
      ROS_INFO_STREAM("Error! Missing ugv maximum speed!");
      return false;
    }

    if (!rh_.getParam("ugv_goal_channel", ugv_goal_channel_)) {
      ROS_INFO_STREAM("Error! Missing ugv goal channel name!");
      return false;
    }

    /// todo subscribe jackal goal channel

    if (!rh_.getParam("uav_max_speed", uav_max_speed_)) {
      ROS_INFO_STREAM("Error! Missing uav maximum speed!");
      return false;
    }

    if (!rh_.getParam("uav_goal_channel", uav_goal_channel_)) {
      ROS_INFO_STREAM("Error! Missing uav goal channel name!");
      return false;
    }

    ROS_INFO_STREAM("Finish loading data!");

    /// todo subscribe pelican goal channel
    return true;
  }

 private:
  ros::NodeHandle nh_, rh_;
  // GroundTruthData ground_truth_data_;
  double convergence_threshold_;

  double ugv_max_speed_;
  std::string ugv_goal_channel_;
  double uav_max_speed_;
  std::string uav_goal_channel_;

  Eigen::MatrixXd ground_truth_location_;
  Eigen::MatrixXd ground_truth_temperature_;
};
}  // namespace sampling

int main(int argc, char ** argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::CentralizedSamplingNode(nh, rh);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
