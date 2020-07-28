#include "sampling_core/sampling_core_performance_evaluation.h"

#include <math.h>

#include "sampling_msgs/SamplingPerformance.h"

namespace sampling {
namespace core {

std::unique_ptr<SamplingCorePerformanceEvaluation>
SamplingCorePerformanceEvaluation::MakeUniqueFromRos(
    ros::NodeHandle &nh, const std::vector<double> &ground_truth_data) {
  if (ground_truth_data.empty()) {
    ROS_ERROR_STREAM("Empty ground truth data for performance evaluation");
    return nullptr;
  }
  return std::unique_ptr<SamplingCorePerformanceEvaluation>(
      new SamplingCorePerformanceEvaluation(nh, ground_truth_data));
}

SamplingCorePerformanceEvaluation::SamplingCorePerformanceEvaluation(
    ros::NodeHandle &nh, const std::vector<double> &ground_truth_data)
    : ground_truth_data_(ground_truth_data), sample_count_(0) {
  event_timer_ = nh.createTimer(
      ros::Duration(1.0),
      &SamplingCorePerformanceEvaluation::ReportPerformanceCallback, this);

  performance_publisher_ = nh.advertise<sampling_msgs::SamplingPerformance>(
      "sampling_performance", 1);
}

bool SamplingCorePerformanceEvaluation::UpdatePerformance(
    const int &sample_count, const std::vector<double> &prediction_data) {
  if (prediction_data.size() != ground_truth_data_.size()) {
    ROS_ERROR_STREAM("Evaluation data size does NOT match!");
    return false;
  }
  sample_count_ = sample_count;
  prediction_data_ = prediction_data;
  if (!CalculateRMSE(ground_truth_data_, prediction_data_, rmse_)) {
    return false;
  }
  return true;
}

bool SamplingCorePerformanceEvaluation::CalculateRMSE(
    const std::vector<double> &ground_truth_data,
    const std::vector<double> &prediction_data, double &rmse) {
  if (ground_truth_data.size() != prediction_data.size() ||
      ground_truth_data.empty())
    return false;
  rmse = 0.0;
  for (int i = 0; i < ground_truth_data.size(); ++i) {
    double error = ground_truth_data[i] - prediction_data[i];
    rmse += (error * error);
  }
  rmse = std::sqrt(rmse / double(ground_truth_data.size()));
  return true;
}

void SamplingCorePerformanceEvaluation::ReportPerformanceCallback(
    const ros::TimerEvent &) {
  if (sample_count_ > 0) {
    sampling_msgs::SamplingPerformance msg;
    msg.header.stamp = ros::Time::now();
    msg.sample_count = sample_count_;
    msg.rmse = rmse_;
    performance_publisher_.publish(msg);
  }
}

}  // namespace core
}  // namespace sampling