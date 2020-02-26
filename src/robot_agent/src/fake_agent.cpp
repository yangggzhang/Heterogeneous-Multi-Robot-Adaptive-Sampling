#include "robot_agent/fake_agent.h"
#include <math.h>
namespace sampling {
namespace agent {
FakeAgentNode::FakeAgentNode(const ros::NodeHandle &nh,
                             const ros::NodeHandle &rh)
    : AgentNode(nh, rh) {
  if (!rh_.getParam("fake_agent_initial_latitude", current_latitude_)) {
    ROS_ERROR("Error! Missing fake_agent_initial_latitude!");
  }

  if (!rh_.getParam("fake_agent_initial_longitude", current_longitude_)) {
    ROS_ERROR("Error! Missing fake_agent_initial_longitude!");
  }

  if (!rh_.getParam("fake_agent_max_vel", max_vel_)) {
    ROS_ERROR("Error! Missing fake agent max vel!");
  }

  if (!rh_.getParam("fake_moving_duration_threshold_s",
                    fake_moving_duration_threshold_s_)) {
    ROS_ERROR("Error! Missing fake agent navigation time threshold!");
  }

  if (!rh_.getParam("fake_distance_threshold_s", fake_distance_threshold_s_)) {
    ROS_ERROR("Error! Missing fake agent navigation distance threshold!");
  }

  if (!rh_.getParam("nagivate_loop_rate", nagivate_loop_rate_int_)) {
    ROS_ERROR("Error! Missing loop rate during navigation!");
  }

  if (!rh_.getParam("observation_noise_std", observation_noise_std_)) {
    ROS_ERROR("Error! Missing observation noise std!");
  }

  if (!rh_.getParam("obstacle_avoidance", obstacle_avoidance_)) {
    ROS_ERROR("Error! Missing obstacle avoidance!");
  }

  if (!rh_.getParam("speed_resolution", speed_resolution_)) {
    ROS_ERROR("Error! Missing speed_resolution!");
  }

  if (!rh_.getParam("orientation_resolution", angle_resolution_)) {
    ROS_ERROR("Error! Missing orientation_resolution!");
  }

  if (!rh_.getParam("collision_radius", collsion_radius_)) {
    ROS_ERROR("Error! Missing collision_radius!");
  }

  if (!rh_.getParam("prediction_time", prediction_time_)) {
    ROS_ERROR("Error! Missing prediction_time!");
  }

  if (!rh_.getParam("goal_cost_gain", goal_cost_gain_)) {
    ROS_ERROR("Error! Missing goal cost gain!");
  }

  if (!rh_.getParam("obstacle_cost_gain", obstacle_cost_gain_)) {
    ROS_ERROR("Error! Missing obstacle cost gain!");
  }

  std::vector<double> mu, sigma, obstacles;
  if (!rh_.getParam("ground_truth_mu", mu)) {
    ROS_ERROR("Error! Missing ground truth mu !");
  }

  if (!rh_.getParam("ground_truth_sig", sigma)) {
    ROS_ERROR("Error! Missing ground truth sigma!");
  }

  if (!rh_.getParam("gaussian_weights", gt_weights_)) {
    ROS_ERROR("Error! Missing gaussian_weights!");
  }

  if (!rh_.getParam("obstacle_pos", obstacles)) {
    ROS_ERROR("Error! Missing obstacle_pos!");
  }
  assert(mu.size() == sigma.size());
  assert((mu.size() % 2) == 0);  // can reshape to n*2 (mu_x, mu_y)
  gt_num_gaussian_ = ((int)mu.size()) / 2;
  gt_mu_.resize(gt_num_gaussian_, 2);
  gt_sigma_.resize(gt_num_gaussian_, 2);
  Eigen::Matrix2d cov;

  for (int i = 0; i < gt_num_gaussian_; i++) {
    for (int j = 0; j < 2; j++) {
      int count = i * 2 + j;
      gt_mu_(i, j) = mu[count];
      gt_sigma_(i, j) = sigma[count];
    }
  }

  assert((obstacles.size() % 2) == 0);  // can reshape to n*2
  num_obstacles_ = ((int)obstacles.size()) / 2;
  obstacle_pos_.resize(num_obstacles_, 2);
  for (int i = 0; i < num_obstacles_; i++) {
    obstacle_pos_.row(i) << obstacles[2 * i], obstacles[2 * i + 1];
  }
  std::cout << obstacle_pos_ << std::endl;
  ROS_INFO_STREAM("Finish Fake Agent Loading!");
}

bool FakeAgentNode::update_goal_from_gps() {
  // fake node
  return true;
}

bool FakeAgentNode::navigate() {
  if (move_to_goal()) {
    return true;
  } else {
    ROS_INFO_STREAM("Robot "
                    << agent_id_
                    << " failed to reach the target location with state ("
                    << current_latitude_ << ", " << current_longitude_ << ")");
    return false;
  }
}

bool FakeAgentNode::move_to_goal() {
  double dist_to_goal =
      sqrt(pow((goal_rtk_latitude_ - current_latitude_), 2) +
           pow((goal_rtk_longitude_ - current_longitude_), 2));
  double vel_dir_x = (goal_rtk_latitude_ - current_latitude_) / dist_to_goal;
  double vel_dir_y = (goal_rtk_longitude_ - current_longitude_) / dist_to_goal;
  std::vector<double> best_vel;
  ros::Time begin = ros::Time::now();
  ros::Time previous = ros::Time::now();
  ros::Duration totalNavigationTime = ros::Time::now() - begin;
  ros::Duration dt = ros::Time::now() - previous;
  ros::Rate navigate_loop_rate(nagivate_loop_rate_int_);
  while (ros::ok() &&  // ros is still alive
         totalNavigationTime.toSec() <= fake_moving_duration_threshold_s_) {
    if (goal_reached()) {
      return true;
    }
    ros::spinOnce();
    best_vel = get_best_vel();
    navigate_loop_rate.sleep();
    // update new position
    dt = ros::Time::now() - previous;
    std::cout << dt << std::endl;
    current_latitude_ += best_vel[0] * dt.toSec();
    current_longitude_ += best_vel[1] * dt.toSec();
    ROS_INFO_STREAM("Robot " << agent_id_ << " current location with state ("
                             << current_latitude_ << ", " << current_longitude_
                             << ")");
    previous = ros::Time::now();
    totalNavigationTime = ros::Time::now() - begin;
  }
  return false;
}

bool FakeAgentNode::goal_reached() {
  return ((sqrt(pow((goal_rtk_latitude_ - current_latitude_), 2) +
                pow((goal_rtk_longitude_ - current_longitude_), 2))) <
          fake_distance_threshold_s_);
}

std::vector<double> FakeAgentNode::get_best_vel() {
  double min_cost = FLT_MAX;
  double best_vx = 0;
  double best_vy = 0;
  double best_goal_cost;
  double best_obstacle_cost;
  for (double speed = 0.0; speed < max_vel_; speed += speed_resolution_) {
    for (double angle = -M_PI; angle < M_PI; angle += angle_resolution_) {
      double v_x = speed * cos(angle);
      double v_y = speed * sin(angle);
      Eigen::MatrixXd traj = get_trajectory(v_x, v_y);
      double goal_cost = goal_cost_gain_ * get_goal_cost(traj);
      double obstacle_cost = obstacle_cost_gain_ * get_obstacle_cost(traj);
      double total_cost = goal_cost + obstacle_cost;
      if (min_cost >= total_cost) {
        min_cost = total_cost;
        best_vx = v_x;
        best_vy = v_y;
        best_goal_cost = goal_cost;
        best_obstacle_cost = obstacle_cost;
      }
    }
  }
  std::vector<double> best_vel{best_vx, best_vy};
  std::cout << "cost: " << best_goal_cost << "," << best_obstacle_cost
            << std::endl;
  return best_vel;
}

Eigen::MatrixXd FakeAgentNode::get_trajectory(double v_x, double v_y) {
  double dt = (1.0 / (float)nagivate_loop_rate_int_);
  double x = current_latitude_;
  double y = current_longitude_;
  int traj_size = (int)prediction_time_ / dt;
  Eigen::MatrixXd traj(traj_size, 2);
  for (int i = 0; i < traj_size; i++) {
    x += v_x * dt;
    y += v_y * dt;
    traj.row(i) << x, y;
  }
  return traj;
}

double FakeAgentNode::get_goal_cost(Eigen::MatrixXd traj) {
  double cost;
  double end_lat = traj((traj.rows() - 1), 0);
  double end_lon = traj((traj.rows() - 1), 1);
  cost = sqrt(pow((goal_rtk_latitude_ - end_lat), 2) +
              pow((goal_rtk_longitude_ - end_lon), 2));
  return cost;
}

double FakeAgentNode::get_obstacle_cost(Eigen::MatrixXd traj) {
  double cost;
  if (obstacle_avoidance_) {
    Eigen::MatrixXd distance_to_obs(traj.rows(), obstacle_pos_.rows());
    for (int i = 0; i < traj.rows(); i++) {
      for (int j = 0; j < obstacle_pos_.rows(); j++) {
        distance_to_obs(i, j) =
            sqrt(pow((obstacle_pos_(j, 0) - traj(i, 0)), 2) +
                 pow((obstacle_pos_(j, 1) - traj(i, 1)), 2));
      }
    }
    Eigen::MatrixXd::Index minRow, minCol;
    double min = distance_to_obs.minCoeff(&minRow, &minCol);
    // std::cout << "Index" << minRow << "," << minCol << std::endl;
    if (min < collsion_radius_) {
      cost = FLT_MAX;
    } else {
      cost = 1.0 / min;
    }
  } else {  // not consider obstacle cost for UAV
    cost = 0;
  }
  return cost;
}

// bool FakeAgentNode::collect_temperature_sample() {
//   sampling_msgs::RequestGroundTruthTemperature srv;
//   srv.request.latitude = current_latitude_;
//   srv.request.longitude = current_longitude_;
//   if (temperature_measurement_client_.call(srv)) {
//     temperature_measurement_ = srv.response.temperature;
//     // add noise:
//     std::normal_distribution<float> dist(
//         0, observation_noise_std_);  // mean followed by stdiv
//     temperature_measurement_ += dist(generator);
//     return true;
//   } else {
//     ROS_INFO_STREAM("Robot " << agent_id_
//                              << " failed to receive temperature
//                              measurement!");
//     return false;
//   }
// }

bool FakeAgentNode::collect_temperature_sample() {
  temperature_measurement_ = getGroundTruth();
  // add noise:
  std::normal_distribution<float> dist(
      0, observation_noise_std_);  // mean followed by stdiv
  temperature_measurement_ += dist(generator);
  return true;
}

double FakeAgentNode::getGroundTruth() {
  Eigen::Vector2d location;
  location << current_latitude_, current_longitude_;
  double total_value = 0;
  for (int i = 0; i < gt_num_gaussian_; i++) {
    total_value += gt_weights_[i] * getPdf(location, gt_mu_.row(i),
                                           gt_sigma_.row(i).asDiagonal());
  }
  return total_value;
}

double FakeAgentNode::getPdf(const Eigen::VectorXd &x,
                             const Eigen::VectorXd &meanVec,
                             const Eigen::MatrixXd &covMat) {
  // compile time:
  const double logSqrt2Pi = 0.5 * std::log(2 * M_PI);
  typedef Eigen::LLT<Eigen::MatrixXd> Chol;
  Chol chol(covMat);
  // Handle non positive definite covariance somehow:
  if (chol.info() != Eigen::Success) throw "decomposition failed!";
  const Chol::Traits::MatrixL &L = chol.matrixL();
  double quadform = (L.solve(x - meanVec)).squaredNorm();
  return std::exp(-x.rows() * logSqrt2Pi - 0.5 * quadform) / L.determinant();
}

}  // namespace agent
}  // namespace sampling