/**
 * AUTHOR: Yang Zhang
 */

#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <boost/functional/hash.hpp>
#include <string>
#include <unordered_map>
#include <utility>  // std::pair, std::make_pair
#include <vector>

namespace sampling {
namespace learning {

const double KLearningBeta = 0.5;

const std::string KLearningType_Greedy = "GREEDY";
const std::string KLearningType_UCB = "UCB";
const std::string KLearningType_Default = KLearningType_Greedy;

class OnlineLearningHandler {
 public:
  OnlineLearningHandler() = delete;

  static std::unique_ptr<OnlineLearningHandler> MakeUniqueFromRosParam(
      ros::NodeHandle &ph);

  bool UpdateSampleCount(const geometry_msgs::Point &position);

  bool InformativeSelection(const Eigen::MatrixXd &locations,
                            const std::vector<double> &mean,
                            const std::vector<double> &variance,
                            geometry_msgs::Point &informative_point);

 private:
  OnlineLearningHandler(const std::string &learning_type,
                        const double &learning_beta);

  std::unordered_map<std::pair<double, double>, double,
                     boost::hash<std::pair<double, double>>>
      count_map_;

  std::string learning_type_;

  double learning_beta_;
};
}  // namespace learning
}  // namespace sampling