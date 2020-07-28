#pragma once

#include <ros/ros.h>

#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace sampling {
namespace core {

class SamplingCorePerformanceEvaluation {
 public:
  SamplingCorePerformanceEvaluation();

  static std::unique_ptr<SamplingCorePerformanceEvaluation> MakeUniqueFromRos(
      ros::NodeHandle &nh, const std::vector<double> &ground_truth_data);

  bool UpdatePerformance(const int &sample_count,
                         const std::vector<double> &prediction_data);

 private:
  SamplingCorePerformanceEvaluation(
      ros::NodeHandle &nh, const std::vector<double> &ground_truth_data);

  bool CalculateRMSE(const std::vector<double> &ground_truth_data,
                     const std::vector<double> &prediction_data, double &rmse);

  void ReportPerformanceCallback(const ros::TimerEvent &);

  ros::Timer event_timer_;

  ros::Publisher performance_publisher_;

  std::vector<double> ground_truth_data_;

  std::vector<double> prediction_data_;

  int sample_count_;

  double rmse_;

};  // namespace scene
}  // namespace core
}  // namespace sampling