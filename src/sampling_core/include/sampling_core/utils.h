
/**
 * Utility functions for sampling core
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <string>

namespace sampling {
namespace utils {

struct gps_location {
  double latitude;
  double longitude;
};

struct map_location {
  double x;
  double y;
};

bool load_ground_truth_data(const std::string &location_data_path,
                            const std::string &temperature_data_path,
                            Eigen::MatrixXd &location,
                            Eigen::MatrixXd &temperature);
}
}