#pragma once

#include <geometry_msgs/PointStamped.h>
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


  double getPoly(double x, double y);

  //   void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg)
  //   override;

 private:
  bool goal_reached_;
  bool obstacle_avoidance_;
  bool random_initial_pos_;
  int gt_num_gaussian_;
  int num_speed_premitive_;
  int groundtruth_type_;
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
  double battery_life_;
  ros::Time robot_start_time_;
  ros::Duration robot_alive_duration_;
  std::default_random_engine generator;
  Eigen::MatrixXd gt_mu_, gt_sigma_, obstacle_pos_;
  std::vector<double> gt_weights_;
  std::vector<double> poly_coeff_;
  std::vector<double> map_range_;
  double map_resolution_;
};
}  // namespace agent
}  // namespace sampling