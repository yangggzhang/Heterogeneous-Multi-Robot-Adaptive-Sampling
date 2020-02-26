/**
 * Utility functions for high dimensio voronoi construction
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

  // Normal 2D Voronoi map
  Voronoi(const Eigen::MatrixXd &location);

  Voronoi(const Eigen::MatrixXd &location,
          const std::vector<std::vector<double>> &motion_primitives);

  bool UpdateVoronoiMap(const Eigen::MatrixXd &agent_locations,
                        const Eigen::VectorXd &scale_factor,
                        std::vector<std::vector<int>> &labels,
                        Eigen::MatrixXd &distance_matrix);

  double L1_Distance(const std::vector<double> &lhs,
                     const std::vector<double> &rhs);

  double L2_Distance(const std::vector<double> &lhs,
                     const std::vector<double> &rhs);

  // Use motion primitive and euclidean distance to calculate high dimentional
  // distance.
  inline double Continuous_Distance(const double &motion_primitive,
                                    const double &euclidean_distance);

  Eigen::MatrixXd GetLocation();

 private:
  Eigen::MatrixXd location_;
  std::vector<std::vector<double>> motion_primitives_;
};
}  // namespace voronoi
}  // namespace sampling