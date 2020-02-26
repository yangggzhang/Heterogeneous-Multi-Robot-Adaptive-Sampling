/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace sampling {
enum HeterogenitySpace { DISTANCE, SPEED, BATTERYLIFE, MOBILITY };

namespace voronoi {
class Voronoi {
 private:
  double L1Distance(const std::vector<double> &lhs,
                    const std::vector<double> &rhs);

  double L2Distance(const std::vector<double> &lhs,
                    const std::vector<double> &rhs);

  // Use motion primitive and euclidean distance to calculate high dimentional
  // distance.
  inline double ContinuousDistance(const double &motion_primitive,
                                   const double &euclidean_distance);

  double HeteroDistance(const std::vector<HeterogenitySpace> &hetero_space,
                        const std::vector<double> &motion_primitive,
                        const double &euclidean_distance);

  // agent_locations is a N x 2 matrix, where N is the number of agents.
  // Returnn an m x n distance map, where map(i,j) represents robot_j 's
  // distance to location_i
  Eigen::MatrixXd GetDistanceMap(const Eigen::MatrixXd &agent_locations);

  // agent_locations is a N x 2 matrix, where N is the number of agents.
  bool IsAgentClosest(const std::vector<HeterogenitySpace> &hetero_space,
                      const std::vector<std::vector<double>> &motion_primitives,
                      const Eigen::VectorXd &distance_vec, const int &agent_id);

  int FindClosestAgent(
      const std::vector<HeterogenitySpace> &hetero_space,
      const std::vector<std::vector<double>> &motion_primitives,
      const Eigen::VectorXd &distance_vec);

 public:
  Voronoi();

  // Normal 2D Voronoi map
  Voronoi(const Eigen::MatrixXd &location);

  Voronoi(const Eigen::MatrixXd &location, const int &num_robots,
          const std::vector<HeterogenitySpace> &hetero_space,
          const std::vector<double> &scale_factors,
          const std::vector<std::vector<double>> &motion_primitives);

  Eigen::MatrixXd GetVoronoiCell(const Eigen::MatrixXd &agent_locations,
                                 const int &agent_id);

  std::vector<Eigen::MatrixXd> GetVoronoiDiagram(
      const Eigen::MatrixXd &agent_locations);

  bool UpdateVoronoiMap(const Eigen::MatrixXd &agent_locations,
                        const Eigen::VectorXd &scale_factor,
                        std::vector<std::vector<int>> &labels,
                        Eigen::MatrixXd &distance_matrix);

  Eigen::MatrixXd GetLocation();

 private:
  Eigen::MatrixXd location_;
  int num_robots_;
  std::vector<double> scale_factors_;
  std::vector<std::vector<double>> motion_primitives_;
  std::vector<HeterogenitySpace> hetero_space_;
};
}  // namespace voronoi
}  // namespace sampling