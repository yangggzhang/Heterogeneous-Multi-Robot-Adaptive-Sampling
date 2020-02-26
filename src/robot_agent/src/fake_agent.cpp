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
    ROS_ERROR("Error! Missing observation noise std!");
  }
  assert(mu.size() == sigma.size());
  assert((mu.size() % 2) == 0);  // can reshape to n*2 (mu_x, mu_y)
  gt_num_gaussian_ = ((int)mu.size()) / 2;
  gt_mu_.resize(gt_num_gaussian_, 2);
  gt_sigma_.resize(gt_num_gaussian_, 2);

  for (int i = 0; i < gt_num_gaussian_; i++) {
    for (int j = 0; j < 2; j++) {
      int count = i * 2 + j;
      gt_mu_(i, j) = mu[count];
      gt_sigma_(i, j) = sigma[count];
    }
  }
  // std::cout << gt_mu_ << std::endl;
  // std::cout << gt_sigma_ << std::endl;
  // create groundtruth distribution
  // for (int i = 0; i<gt_num_gaussian_; i++)
  // {

  // }
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

bool FakeAgentNode::collect_temperature_sample() {
  sampling_msgs::RequestGroundTruthTemperature srv;
  srv.request.latitude = current_latitude_;
  srv.request.longitude = current_longitude_;
  if (temperature_measurement_client_.call(srv)) {
    temperature_measurement_ = srv.response.temperature;
    // add noise:
    std::normal_distribution<float> dist(
        0, observation_noise_std_);  // mean followed by stdiv
    temperature_measurement_ += dist(generator);
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << agent_id_
                             << " failed to receive temperature measurement!");
    return false;
  }
}

}  // namespace agent
}  // namespace sampling