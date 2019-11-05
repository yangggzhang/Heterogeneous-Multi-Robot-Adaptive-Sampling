/**
 * Utility functions for voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace sampling {
namespace voronoi {
class Voronoi {
 public:
  Voronoi();

  Voronoi(const Eigen::MatrixXd &location);

  bool UpdateVoronoiMap(const Eigen::MatrixXd &agent_locations,
                        const Eigen::VectorXd &scale_factor,
                        std::vector<std::vector<int>> &labels,
                        Eigen::MatrixXd &distance_matrix);

  Eigen::MatrixXd GetLocation();

 private:
  Eigen::MatrixXd location_;
};
}  // namespace voronoi
}  // namespace sampling