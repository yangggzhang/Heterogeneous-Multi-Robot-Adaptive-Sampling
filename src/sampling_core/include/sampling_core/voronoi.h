/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

template <>
struct std::hash<std::pair<double, double>> {
  inline size_t operator()(const std::pair<double, double> &v) const {
    std::hash<double> double_hasher;
    return double_hasher(v.first) ^ double_hasher(v.second);
  }
};

namespace sampling {
enum HeterogenitySpace { DISTANCE, SPEED, BATTERYLIFE, MOBILITY, REACHABILITY };

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

  double HeteroDistance(
      const std::vector<HeterogenitySpace> &hetero_space,
      const std::vector<double> &scale_factor,
      const std::vector<double> &motion_primitive,
      const double &euclidean_distance, const Eigen::VectorXd &grid_location,
      const std::unordered_set<std::pair<double, double>> &unreachable_points);

  // agent_locations is a N x 2 matrix, where N is the number of agents.
  // Returnn an m x n distance map, where map(i,j) represents robot_j 's
  // distance to location_i
  Eigen::MatrixXd GetDistanceMap(const Eigen::MatrixXd &agent_locations);

  // agent_locations is a N x 2 matrix, where N is the number of agents.
  bool IsAgentClosest(const std::vector<HeterogenitySpace> &hetero_space,
                      const std::vector<std::vector<double>> &motion_primitives,
                      const Eigen::VectorXd &distance_vec,
                      const Eigen::VectorXd &grid_location,
                      const int &agent_id);

  int FindClosestAgent(
      const std::vector<HeterogenitySpace> &hetero_space,
      const std::vector<std::vector<double>> &motion_primitives,
      const Eigen::VectorXd &distance_vec,
      const Eigen::VectorXd &grid_location);

 public:
  Voronoi();

  // Normal 2D Voronoi map
  Voronoi(const Eigen::MatrixXd &location);

  Voronoi(const Eigen::MatrixXd &location, const int &num_robots,
          const std::vector<HeterogenitySpace> &hetero_space,
          const std::vector<double> &scale_factors,
          const std::vector<std::vector<double>> &motion_primitives);

  std::vector<int> GetSingleVoronoiCellIndex(
      const Eigen::MatrixXd &agent_locations, const int &agent_id);

  Eigen::MatrixXd GetSingleVoronoiCell(const Eigen::MatrixXd &agent_locations,
                                       const int &agent_id);

  std::vector<Eigen::MatrixXd> GetVoronoiCells(
      const Eigen::MatrixXd &agent_locations);

  void UpdateUnreachableLocations(
      const std::vector<Eigen::MatrixXd> &unreachable_locations);

  std::vector<int> GetVoronoiIndex(const Eigen::MatrixXd &agent_locations);

 private:
  Eigen::MatrixXd location_;
  int num_robots_;
  std::vector<double> scale_factors_;
  std::vector<std::vector<double>> motion_primitives_;
  std::vector<HeterogenitySpace> hetero_space_;
  std::vector<std::unordered_set<std::pair<double, double>>>
      unreachable_locations_;
};
}  // namespace voronoi
}  // namespace sampling