#include "sampling_core/utils.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace sampling {
namespace utils {
bool load_ground_truth_data(const std::string &location_data_path,
                            const std::string &temperature_data_path,
                            Eigen::MatrixXd &location,
                            Eigen::MatrixXd &temperature) {
  std::ifstream finFss(temperature_data_path.c_str(), std::ifstream::in);

  if (!finFss.is_open()) {
    ROS_INFO_STREAM("open Fss File: Error opening file");
    return false;
  }

  std::vector<double> Fss_vec;
  std::string line;

  while (getline(finFss, line)) {
    std::istringstream sin(line);

    std::string field;
    double a;

    while (getline(sin, field, ',')) {
      a = std::stod(field);
      Fss_vec.push_back(a);
    }
  }
  temperature.resize(Fss_vec.size(), 1);
  for (int i = 0; i < Fss_vec.size(); i++) {
    temperature(i, 0) = Fss_vec[i];
  }

  std::ifstream finXss(location_data_path.c_str(), std::ifstream::in);

  if (!finXss.is_open()) {
    ROS_INFO_STREAM("open Xss File: Error opening file");
    return false;
  }

  std::vector<double> Xss_x;
  std::vector<double> Xss_y;
  std::string line1;

  while (getline(finXss, line1)) {
    std::istringstream sin(line1);

    std::string field;
    double a;
    int n = 0;

    while (getline(sin, field, ',')) {
      a = std::stod(field);
      if (n == 0) {
        Xss_x.push_back(a);
        n = n + 1;
      } else if (n == 1) {
        Xss_y.push_back(a);
      }
    }
  }
  location.resize(Xss_x.size(), 2);
  for (int i = 0; i < Xss_x.size(); i++) {
    location(i, 0) = Xss_x[i];
    location(i, 1) = Xss_y[i];
  }
  return true;
}
}  // namespace utils
}  // namespace sampling
