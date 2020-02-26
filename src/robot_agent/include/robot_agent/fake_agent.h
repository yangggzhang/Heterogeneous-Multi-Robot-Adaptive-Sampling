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

  Eigen::MatrixXd get_trajectory(double v_x, double v_y);

  std::vector<double> get_best_vel();

  double get_goal_cost(Eigen::MatrixXd traj);

  double get_obstacle_cost(Eigen::MatrixXd traj);

  bool collect_temperature_sample() override;

  double getGroundTruth();
  double getPdf(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec,
                const Eigen::MatrixXd &covMat);
  //   void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg)
  //   override;

 private:
  bool goal_reached_;
  bool obstacle_avoidance_;
  int gt_num_gaussian_;
  double fake_moving_duration_threshold_s_;
  double fake_distance_threshold_s_;
  double observation_noise_std_;
  double max_vel_;
  double nagivate_loop_rate_int_;
  double speed_resolution_;
  double angle_resolution_;
  double prediction_time_;
  double num_obstacles_;
  double collsion_radius_;
  double goal_cost_gain_;
  double obstacle_cost_gain_;
  std::default_random_engine generator;
  Eigen::MatrixXd gt_mu_, gt_sigma_, obstacle_pos_;
  std::vector<double> gt_weights_;
};
}  // namespace agent
}  // namespace sampling