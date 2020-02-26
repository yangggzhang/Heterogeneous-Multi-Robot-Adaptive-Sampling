#pragma once

#include <geometry_msgs/PointStamped.h>
#include <sampling_msgs/RequestGroundTruthTemperature.h>
#include <Eigen/Dense>
#include <random>
#include "robot_agent/robot_agent.h"

namespace sampling {
namespace agent {

class FakeAgentNode : public AgentNode {
 public:
  FakeAgentNode(){};

  FakeAgentNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  bool update_goal_from_gps();

  bool navigate();

  bool move_to_goal();

  bool goal_reached();

  bool collect_temperature_sample() override;

  //   void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg)
  //   override;

 private:
  int gt_num_gaussian_;
  double fake_moving_duration_threshold_s_;
  double fake_distance_threshold_s_;
  double observation_noise_std_;
  double max_vel_;
  bool goal_reached_;
  double nagivate_loop_rate_int_;
  std::default_random_engine generator;
  Eigen::MatrixXd gt_mu_, gt_sigma_;
};
}  // namespace agent
}  // namespace sampling