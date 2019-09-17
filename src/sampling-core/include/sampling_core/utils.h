
/**
 * Utility functions for sampling core
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <string>

namespace sampling {
namespace utils {
bool load_ground_truth_data(const std::string &location_data_path,
                            const std::string &temperature_data_path,
                            Eigen::MatrixXd &location,
                            Eigen::MatrixXd &temperature);
}
}