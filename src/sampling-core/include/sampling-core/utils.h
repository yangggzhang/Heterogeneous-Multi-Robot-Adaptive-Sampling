
/**
 * Utility functions for sampling core
 * AUTHOR: Yang Zhang
 */

#pragma once
namespace sampling {
bool load_ground_truth_data(const std::string &location_data_path,
                            const std::string &temperature_data_path,
                            Eigen::MatrixXd &location,
                            Eigen::MatrixXd &temperature);
}