#include "sampling_core/sampling_core_params.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <algorithm>  // std::random_shuffle
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "sampling_msgs/Sample.h"
#include "sampling_utils/utils.h"

namespace sampling {
namespace core {

SamplingCoreParams::SamplingCoreParams() {}

bool SamplingCoreParams::LoadFromRosParams(ros::NodeHandle &ph) {
  std::string test_location_file;
  if (!ph.getParam("test_location_file", test_location_file)) {
    ROS_ERROR_STREAM("Please provide test locations for sampling task!");
    return false;
  }
  std::string pack_path = ros::package::getPath(KDataPackage);
  std::string test_location_dir = pack_path + "/location/" + test_location_file;

  if (!LoadMactrix(test_location_dir, test_locations)) {
    ROS_ERROR_STREAM("Failed to load test locations for sampling!");
    return false;
  }

  MatrixToMsg(test_locations, test_locations_msg);

  if (!ph.getParam("agent_ids", agent_ids)) {
    ROS_ERROR_STREAM("Please provide agent ids for sampling task!");
    return false;
  }

  std::string groundtruth_measurement_file;
  if (!ph.getParam("groundtruth_measurement_file",
                   groundtruth_measurement_file)) {
    ROS_WARN_STREAM(
        "Ground truth meansurement file is NOT provided! Please provide "
        "samples for model initialization!");
    have_groundtruth_measurement = false;
    std::string initial_measurement_file;
    if (!ph.getParam("initial_measurement_file", initial_measurement_file)) {
      ROS_ERROR_STREAM(
          "Please provide initial measurements for sampling task!");
      return false;
    }
    std::string initial_measurement_dir =
        pack_path + "/measurement/" + initial_measurement_file;
    if (!LoadVector(initial_measurement_dir, initial_measurements)) {
      ROS_ERROR_STREAM("Failed to load initial measurements for sampling!");
      return false;
    }
    std::string initial_location_file;
    if (!ph.getParam("initial_location_file", initial_location_file)) {
      ROS_ERROR_STREAM("Please provide initial locations for sampling task!");
      return false;
    }
    std::string initial_location_dir =
        pack_path + "/location/" + initial_location_file;
    if (!LoadMactrix(initial_location_dir, initial_locations)) {
      ROS_ERROR_STREAM("Failed to load initial locations for sampling!");
      return false;
    }
    MatrixToMsg(initial_locations, initial_locations_msg);

    if (initial_measurements.rows() != initial_measurements.size()) {
      ROS_ERROR_STREAM("Initial locations and measurements do NOT match!");
      return false;
    }
  } else {
    int initial_sample_size =
        std::max(KInitSampleSize,
                 int(KInitSampleRatio * ground_truth_measurements.size()));
    std::vector<int> index_vec;
    for (int i = 0; i < ground_truth_measurements.size(); ++i)
      index_vec.push_back(i);
    // using built-in random generator:
    std::random_shuffle(index_vec.begin(), index_vec.end());

    std::vector<int> random_initial_index;
    random_initial_index.reserve(initial_sample_size);
    for (int i = 0; i < initial_sample_size; ++i) {
      random_initial_index.push_back(index_vec[i]);
    }

    if (!utils::ExtractRows(test_locations, random_initial_index,
                            initial_locations)) {
      ROS_ERROR_STREAM("Failed to generate initial locations for sampling!");
      return false;
    }
    MatrixToMsg(initial_locations, initial_locations_msg);

    if (!utils::ExtractVec(ground_truth_measurements, random_initial_index,
                           initial_measurements)) {
      ROS_ERROR_STREAM("Failed to generate initial measurements for sampling!");
      return false;
    }
  }

  have_groundtruth_measurement = true;

  if (!ph.getParam("model_update_frequency_count",
                   model_update_frequency_count)) {
    ROS_WARN_STREAM("Using default model update frequency (count) : "
                    << KModelUpdateFrequencyCount);
    model_update_frequency_count = KModelUpdateFrequencyCount;
    return false;
  }

  return true;
}

bool SamplingCoreParams::LoadMactrix(const std::string &path,
                                     Eigen::MatrixXd &data) {
  std::ifstream file(path.c_str(), std::ifstream::in);

  if (!file.is_open()) {
    ROS_INFO_STREAM("Error opening file" << path);
    return false;
  }

  std::vector<std::vector<double>> data_vec;

  std::string new_line;

  while (getline(file, new_line)) {
    std::stringstream ss(new_line);
    std::vector<double> new_data;
    for (double k; ss >> k;) {
      new_data.push_back(k);
      if (ss.peek() == ',') ss.ignore();
    }
    data_vec.push_back(new_data);
  }

  if (data_vec.empty()) {
    ROS_ERROR_STREAM("Empty data!");
    return false;
  }

  data.resize(data_vec.size(), data_vec.front().size());
  for (int i = 0; i < data_vec.size(); ++i) {
    for (int j = 0; j < data_vec[i].size(); ++j) {
      data(i, j) = data_vec[i][j];
    }
  }
  return true;
}

bool SamplingCoreParams::LoadVector(const std::string &path,
                                    Eigen::VectorXd &data) {
  Eigen::MatrixXd data_mat;
  if (!LoadMactrix(path, data_mat)) {
    ROS_ERROR_STREAM("Failed to load " << path);
    return false;
  }
  if (data_mat.cols() != 1) {
    ROS_ERROR_STREAM("Loaded data has multiple dimensions!");
    return false;
  }
  data = data_mat.col(0);
  return true;
}

void SamplingCoreParams::MatrixToMsg(const Eigen::MatrixXd &data,
                                     std::vector<geometry_msgs::Point> &msg) {
  msg.resize(data.rows());
  for (int i = 0; i < data.rows(); ++i) {
    geometry_msgs::Point point;
    point.x = data(i, 0);
    point.y = data(i, 1);
    msg[i] = point;
  }
}

}  // namespace core
}  // namespace sampling