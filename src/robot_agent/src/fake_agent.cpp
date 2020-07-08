#include "robot_agent/fake_agent.h"

#include <math.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <unistd.h>

namespace sampling {
namespace agent {
FakeSamplingAgent::FakeSamplingAgent(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &rh)
    : SamplingAgent(nh, rh) {
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

  if (!rh_.getParam("nagivate_loop_rate", nagivate_loop_rate_int_)) {
    ROS_ERROR("Error! Missing loop rate during navigation!");
  }

  if (!rh_.getParam("observation_noise_std", observation_noise_std_)) {
    ROS_ERROR("Error! Missing observation noise std!");
  }

  if (!rh_.getParam("obstacle_avoidance", obstacle_avoidance_)) {
    ROS_ERROR("Error! Missing obstacle avoidance!");
  }

  if (!rh_.getParam("num_speed_premitive", num_speed_premitive_)) {
    ROS_ERROR("Error! Missing num_speed_premitive_!");
  }
  speed_resolution_ = (2 * max_vel_) / (double)num_speed_premitive_;
  fake_distance_threshold_s_ =
      1.5 * (1.0 / (float)nagivate_loop_rate_int_) * max_vel_;

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

  if (!rh_.getParam("ground_truth_type", groundtruth_type_)) {
    ROS_ERROR("Error! Missing ground truth type");
  }

  if (!rh_.getParam("poly_coeff", poly_coeff_)) {
    ROS_ERROR("Error! Missing poly_coeff");
  }
  assert(poly_coeff_.size() == 21);

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

  if (!rh_.getParam("battery_life", battery_life_)) {
    ROS_ERROR("Error! Missing battery_life_!");
  }

  if (!rh_.getParam("random_initial_pos", random_initial_pos_)) {
    ROS_ERROR("Error! Missing random_initial_pos!");
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
  // std::cout << obstacle_pos_ << std::endl;
  robot_start_time_ = ros::Time::now();

  srand(getpid() + time(NULL));
  // random initialize position
  if (random_initial_pos_) {
    if (!rh_.getParam("map_range", map_range_)) {
      ROS_ERROR("Error! Missing map_range!");
    }

    if (!rh_.getParam("map_resolution", map_resolution_)) {
      ROS_ERROR("Error! Missing map_range!");
    }
    double min_lat = map_range_[0] / map_resolution_;
    double min_lng = map_range_[1] / map_resolution_;
    double max_lat = map_range_[2] / map_resolution_;
    double max_lng = map_range_[3] / map_resolution_;

    int lat_range = (int)(max_lat - min_lat + 1);
    int lng_range = (int)(max_lng - min_lng + 1);

    int rand_lat_int = rand() % lat_range + min_lat;
    int rand_lng_int = rand() % lng_range + min_lng;

    double rand_lat_d = ((double)rand_lat_int) * map_resolution_;
    double rand_lng_d = ((double)rand_lng_int) * map_resolution_;

    while (!checkCollision(rand_lat_d, rand_lng_d)) {
      rand_lat_int = rand() % lat_range + min_lat;
      rand_lng_int = rand() % lng_range + min_lng;
      rand_lat_d = ((double)rand_lat_int) * map_resolution_;
      rand_lng_d = ((double)rand_lng_int) * map_resolution_;
    }
    current_latitude_ = rand_lat_d;
    current_longitude_ = rand_lng_d;
  }
  ROS_INFO_STREAM("Finish Fake Agent Loading!");
}

bool FakeSamplingAgent::checkCollision(double lat, double lng) {
  double distance;
  for (int j = 0; j < obstacle_pos_.rows(); j++) {
    distance = sqrt(pow((obstacle_pos_(j, 0) - lat), 2) +
                    pow((obstacle_pos_(j, 1) - lng), 2));
    if (distance <= collsion_radius_) {
      return false;
    }
  }
  return true;
}

bool FakeSamplingAgent::update_goal_from_gps() {
  // fake node
  return true;
}

bool FakeSamplingAgent::navigate() {
  robot_alive_duration_ = ros::Time::now() - robot_start_time_;
  if (robot_alive_duration_.toSec() <= battery_life_) {
    if (move_to_goal()) {
      return true;
    } else {
      ROS_INFO_STREAM(
          "Robot " << agent_id_
                   << " failed to reach the target location with state ("
                   << current_latitude_ << ", " << current_longitude_ << ")");
      return false;
    }
  } else {
    current_latitude_ = -1000;
    current_longitude_ = -1000;
    agent_state_ = DIED;
    ROS_INFO_STREAM("Robot " << agent_id_ << " Battery used up!! Died... TAT ("
                             << current_latitude_ << ", " << current_longitude_
                             << ")");
    return false;
  }
}

bool FakeSamplingAgent::move_to_goal() {
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
    ROS_INFO_STREAM("Robot " << agent_id_ << " current location ("
                             << current_latitude_ << ", " << current_longitude_
                             << ") goal (" << goal_rtk_latitude_ << ", "
                             << goal_rtk_longitude_ << ")");
    previous = ros::Time::now();
    totalNavigationTime = ros::Time::now() - begin;
  }
  return false;
}

bool FakeSamplingAgent::goal_reached() {
  return ((sqrt(pow((goal_rtk_latitude_ - current_latitude_), 2) +
                pow((goal_rtk_longitude_ - current_longitude_), 2))) <
          fake_distance_threshold_s_);
}

std::vector<double> FakeSamplingAgent::get_best_vel() {
  double min_cost = FLT_MAX;
  double best_vx = 0;
  double best_vy = 0;
  double best_goal_cost;
  double best_obstacle_cost;
  for (double speed = 0.0; speed < max_vel_ + speed_resolution_;
       speed += speed_resolution_) {
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
  return best_vel;
}

Eigen::MatrixXd FakeSamplingAgent::get_trajectory(double v_x, double v_y) {
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

double FakeSamplingAgent::get_goal_cost(Eigen::MatrixXd traj) {
  double cost;
  double end_lat = traj((traj.rows() - 1), 0);
  double end_lon = traj((traj.rows() - 1), 1);
  cost = sqrt(pow((goal_rtk_latitude_ - end_lat), 2) +
              pow((goal_rtk_longitude_ - end_lon), 2));
  return cost;
}

double FakeSamplingAgent::get_obstacle_cost(Eigen::MatrixXd traj) {
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

// bool FakeSamplingAgent::collect_temperature_sample() {
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

bool FakeSamplingAgent::collect_temperature_sample() {
  temperature_measurement_ = getGroundTruth();
  // add noise:
  std::normal_distribution<float> dist(
      0, observation_noise_std_);  // mean followed by stdiv
  temperature_measurement_ += dist(generator);
  if (current_latitude_ > map_range_[2] || current_latitude_ < map_range_[0] ||
      current_longitude_ > map_range_[3] ||
      current_longitude_ < map_range_[1]) {
    temperature_measurement_ += 2 * dist(generator);
  }
  return true;
}

double FakeSamplingAgent::getGroundTruth() {
  Eigen::Vector2d location;
  location << current_latitude_, current_longitude_;
  double total_value = 0;
  if (groundtruth_type_ == 0) {
    for (int i = 0; i < gt_num_gaussian_; i++) {
      total_value += gt_weights_[i] * getPdf(location, gt_mu_.row(i),
                                             gt_sigma_.row(i).asDiagonal());
    }
  } else if (groundtruth_type_ == 1) {
    total_value = getPoly(current_latitude_, current_longitude_);
  }
  return total_value;
}

double FakeSamplingAgent::getPdf(const Eigen::VectorXd &x,
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

double FakeSamplingAgent::getPoly(double x, double y) {
  double value;
  value = poly_coeff_[0] + poly_coeff_[1] * x + poly_coeff_[2] * y +
          poly_coeff_[3] * pow(x, 2) + poly_coeff_[4] * x * y +
          poly_coeff_[5] * pow(y, 2) + poly_coeff_[6] * pow(x, 3) +
          poly_coeff_[7] * pow(x, 2) * y + poly_coeff_[8] * x * pow(y, 2) +
          poly_coeff_[9] * pow(y, 3) + poly_coeff_[10] * pow(x, 4) +
          poly_coeff_[11] * pow(x, 3) * y +
          poly_coeff_[12] * pow(x, 2) * pow(y, 2) +
          poly_coeff_[13] * x * pow(y, 3) + poly_coeff_[14] * pow(y, 4) +
          poly_coeff_[15] * pow(x, 5) + poly_coeff_[16] * pow(x, 4) * y +
          poly_coeff_[17] * pow(x, 3) * pow(y, 2) +
          poly_coeff_[18] * pow(x, 2) * pow(y, 3) +
          poly_coeff_[19] * x * pow(y, 4) + poly_coeff_[20] * pow(y, 5);
  return value;
}

}  // namespace agent
}  // namespace sampling