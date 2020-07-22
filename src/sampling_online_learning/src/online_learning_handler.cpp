#include "sampling_online_learning/online_learning_handler.h"

namespace sampling {
namespace learning {

std::unique_ptr<OnlineLearningHandler>
OnlineLearningHandler::MakeUniqueFromRosParam(ros::NodeHandle &ph) {
  std::string learning_type;
  double learning_beta;
  ph.param<std::string>("learning_type", learning_type, KLearningType_Default);
  ph.param<double>("learning_beta", learning_beta, KLearningBeta);
  return std::unique_ptr<OnlineLearningHandler>(
      new OnlineLearningHandler(learning_type, learning_beta));
}

bool OnlineLearningHandler::UpdateSampleCount(
    const geometry_msgs::Point &position) {
  count_map_[std::make_pair(position.x, position.y)] += 1.0;
  return true;
}

bool OnlineLearningHandler::InformativeSelection(
    const Eigen::MatrixXd &locations, const std::vector<double> &mean,
    const std::vector<double> &variance,
    geometry_msgs::Point &informative_point) {
  const size_t location_size = locations.rows();
  if (location_size != mean.size() || location_size != variance.size()) {
    ROS_ERROR_STREAM("Informative point selection data does NOT match!");
    return false;
  }
  if (KLearningType_Greedy.compare(learning_type_) == 0) {
    double max_variance = 0.0;
    int max_variance_index = 0;
    for (int i = 0; i < variance.size(); ++i) {
      if (variance[i] > max_variance) {
        max_variance = variance[i];
        max_variance_index = i;
      }
    }
    informative_point.x = locations(max_variance_index, 0);
    informative_point.y = locations(max_variance_index, 1);
    return true;
  } else if (KLearningType_UCB.compare(learning_type_) == 0) {
    Eigen::VectorXd utility = Eigen::VectorXd(location_size);
    for (int i = 0; i < location_size; ++i) {
      utility(i) =
          mean[i] +
          variance[i] * learning_beta_ /
              (count_map_[std::make_pair(locations(i, 0), locations(i, 1))] +
               1.0);
    }
    int max_utility_index = utility.maxCoeff();
    informative_point.x = locations(max_utility_index, 0);
    informative_point.y = locations(max_utility_index, 1);
    return true;
  } else {
    ROS_ERROR_STREAM("Unknown informative point selection method!");
    return false;
  }

  return false;
}

OnlineLearningHandler::OnlineLearningHandler(const std::string &learning_type,
                                             const double &learning_beta)
    : learning_type_(learning_type), learning_beta_(learning_beta) {}

}  // namespace learning
}  // namespace sampling