#include "gp_utils.h"
namespace sampling {

bool distance_2d(const Eigen::MatrixXd &location_1,
                 const Eigen::MatrixXd &location_2, Eigen::MatrixXd &distance) {
  if (location_1.columns() != 2 || location_2.columns() != 2) {
    return false;
  }
  distance.resize(location_1.rows(), location_2.rows());
  for (int i = 0; i < location_1.rows(); i++) {
    for (int j = 0; j < location_2.rows(); j++) {
      distance(i, j) = (location_1.row(i) - location_2.row(j)).norm();
    }
  }
  return true;
}

bool closest_index(const Eigen::MatrixXd &robots_location,
                   const Eigen::MatrixXd &cell_location,
                   std::vector<std::vector<size_t>> &closest_cell) {
  closest_cell.clear();
  /// robots_location.rows == number of robots
  closest_cell.resize(robots_location.rows());
  Eigen::MatrixXd distance;
  if (!distance_2d(robots_location, cell_location, distance)) {
    return false;
  }
  Eigen::MatrixXd::Index min_index;
  for (int i = 0; i < distance.columns(); i++) {
    distance.col(i).minCoeff(&min_index);
    closest_cell[(int)min_index].push_back(i);
  }
  return true;
}
}  // namespace sampling