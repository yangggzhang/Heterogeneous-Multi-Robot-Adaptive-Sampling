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

  std::vector<double> mu, sigma;
  if (!rh_.getParam("ground_truth_mu", mu)) {
    ROS_ERROR("Error! Missing ground truth mu !");
  }
  if (!rh_.getParam("ground_truth_sig", sigma)) {
    ROS_ERROR("Error! Missing ground truth sigma!");
  }
  if (!rh_.getParam("gaussian_weights", gt_weights_)) {
    ROS_ERROR("Error! Missing gaussian_weights!");
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
    // std::cout << gt_mu_.row(i) << std::endl;
  }
  // std::cout << gt_mu_ << std::endl;
  // std::cout << gt_sigma_ << std::endl;
  // create groundtruth distribution
  // Eigen::Vector2d location;
  // location << current_latitude_, current_longitude_;
  // for (int i = 0; i < gt_num_gaussian_; i++) {
  //   Eigen::Vector2d mu_i = gt_mu_.row(i);
  //   Eigen::Vector2d sigma_i = gt_sigma_.row(i);
  //   std::cout << gt_mu_.row(i) << std::endl;
  //   std::cout << gt_sigma_.row(i).size() << std::endl;
  //   cov = sigma_i.asDiagonal();

  //   std::cout << gt_weights_[i] * getPdf(location, gt_mu_.row(i),
  //                                        gt_sigma_.row(i).asDiagonal())
  //             << std::endl;
  // }
  // test==============================================
  // Eigen::MatrixXd sigmat(2, 2);
  // sigmat << 1, 0.1, 0.1, 1;
  // Eigen::VectorXd mean(2);
  // mean << 0, 0;
  // Eigen::VectorXd test(2);
  // test << 0, 0;
  // std::cout << getPdf(test, mean, sigmat) << std::endl; //0.16
  // test << 0.6, 0.6;
  // std::cout << getPdf(test, mean, sigmat) << std::endl;//0.1153

  ROS_INFO_STREAM("Finish Fake Agent Loading!");
  //   ROS_INFO_STREAM("Jackal move base server came up! READY TO GO!!!");
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
  ros::Time begin = ros::Time::now();
  ros::Time previous = ros::Time::now();
  ros::Duration totalNavigationTime = ros::Time::now() - begin;
  ros::Duration dt = ros::Time::now() - previous;
  ros::Rate navigate_loop_rate(nagivate_loop_rate_int_);
  while (ros::ok() &&  // ros is still alive
         totalNavigationTime.toSec() <= fake_moving_duration_threshold_s_) {
    ros::spinOnce();
    navigate_loop_rate.sleep();
    // update new position
    dt = ros::Time::now() - previous;
    current_latitude_ += max_vel_ * vel_dir_x * dt.toSec();
    current_longitude_ += max_vel_ * vel_dir_y * dt.toSec();
    if (goal_reached()) {
      return true;
    }
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