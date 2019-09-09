#include "gp_utils.h"
#include "sampling_visualization.h"
#include <ros/ros.h>
#include <string>

namespace sampling {
class CentralizedSamplingNode {
public:
  CentralizedSamplingNode(const ros::NodeHandle &nh, cosnt ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    load_parameter();
  }

  bool load_parameter() {
    if (!rh_.getParam("ground_truth_data_path", ground_truth_data_path_)) {
      ROS_INFO_STREAM("Error! Missing ground truth temperature data!");
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

    /// todo subscribe pelican goal channel
    if ()
      return true;
  }

private:
  ros::NodeHandle nh_, rh_;
  std::string ground_truth_data_path_;
  GroundTruthData ground_truth_data_;
  double convergence_threshold_;

  double ugv_max_speed_;
  std::string ugv_goal_channel_;
  double uav_max_speed_;
  std::string uav_goal_channel_;
};
}

int main() {
  ros::NodeHandle nh, rh("~");
  return 0;
}
