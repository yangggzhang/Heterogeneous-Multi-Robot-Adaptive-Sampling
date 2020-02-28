#include "sampling_core/voronoi.h"

#include <math.h> /* sqrt */
#include <ros/ros.h>
#include <limits>

namespace sampling {
namespace voronoi {

Voronoi::Voronoi() {}

Voronoi::Voronoi(const Eigen::MatrixXd &location) : location_(location) {}

Voronoi::Voronoi(const Eigen::MatrixXd &location, const int &num_robots,
                 const std::vector<HeterogenitySpace> &hetero_space,
                 const std::vector<double> &scale_factors,
                 const std::vector<std::vector<double>> &motion_primitives)
    : location_(location),
      num_robots_(num_robots),
      hetero_space_(hetero_space),
      scale_factors_(scale_factors),
      motion_primitives_(motion_primitives) {
  unreachable_locations_.resize(num_robots_);
}

double Voronoi::L1Distance(const std::vector<double> &lhs,
                           const std::vector<double> &rhs) {
  assert(lhs.size() == rhs.size());
  double distance = 0.0;
  for (int i = 0; i < lhs.size(); ++i) {
    distance += fabs(lhs[i] - rhs[i]);
  }
  return distance;
}

double Voronoi::L2Distance(const std::vector<double> &lhs,
                           const std::vector<double> &rhs) {
  assert(lhs.size() == rhs.size());
  double distance = 0.0;
  for (int i = 0; i < lhs.size(); ++i) {
    distance += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
  }
  return std::sqrt(distance);
}

inline double Voronoi::ContinuousDistance(const double &motion_primitive,
                                          const double &euclidean_distance) {
  double px = exp(motion_primitive * euclidean_distance);
  double nx = exp(-motion_primitive * euclidean_distance);
  return (px - nx) / (px + nx);
}

Eigen::MatrixXd Voronoi::GetDistanceMap(
    const Eigen::MatrixXd &agent_locations) {
  assert(agent_locations.rows() == num_robots_);
  Eigen::MatrixXd distance_map(location_.rows(), num_robots_);
  double max_distance = 0.0;
  for (int i = 0; i < num_robots_; ++i) {
    Eigen::MatrixXd distance = location_;
    distance.col(0) = distance.col(0).array() - agent_locations(i, 0);
    distance.col(1) = distance.col(1).array() - agent_locations(i, 1);
    distance_map.col(i) = distance_map.col(i).array().abs();
    distance_map.col(i) = distance.rowwise().norm();
    // distance_map.col(i) = distance.rowwise().sum();
    max_distance = std::max(max_distance, distance_map.col(i).maxCoeff());
  }
  return distance_map.array() / max_distance;
}

double Voronoi::HeteroDistance(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<double> &scale_factor,
    const std::vector<double> &motion_primitive,
    const double &euclidean_distance, const Eigen::VectorXd &grid_location,
    const std::unordered_set<std::pair<double, double>,
                             boost::hash<std::pair<double, double>>>
        &unreachable_points) {
  Eigen::VectorXd distance_vec(hetero_space.size());
  for (int i = 0; i < hetero_space.size(); ++i) {
    switch (hetero_space[i]) {
      case DISTANCE: {
        distance_vec(i) = euclidean_distance;
        // distance_vec(i) =
        //     ContinuousDistance(motion_primitive[i], euclidean_distance);
        break;
      }
      case SPEED: {
        distance_vec(i) =
            ContinuousDistance(motion_primitive[i], euclidean_distance);
        break;
      }
      case BATTERYLIFE: {
        distance_vec(i) =
            ContinuousDistance(motion_primitive[i], euclidean_distance);
        break;
      }
      case MOBILITY: {
        distance_vec(i) =
            ContinuousDistance(motion_primitive[i], euclidean_distance);
        break;
      }
      case REACHABILITY: {
        if (unreachable_points.count({grid_location(0), grid_location(1)})) {
          distance_vec(i) = std::numeric_limits<double>::infinity();
          return std::numeric_limits<double>::infinity();
        } else {
          distance_vec(i) = 0;
        }
        break;
      }
      default:
        break;
    }
    distance_vec(i) = distance_vec(i) * scale_factor[i];
  }
  return distance_vec.norm();
}

bool Voronoi::IsAgentClosest(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<std::vector<double>> &motion_primitives,
    const Eigen::VectorXd &distance_vec, const Eigen::VectorXd &grid_location,
    const int &agent_id) {
  assert(distance_vec.size() == num_robots_);
  const double agent_distance = HeteroDistance(
      hetero_space, scale_factors_, motion_primitives_[agent_id],
      distance_vec(agent_id), grid_location, unreachable_locations_[agent_id]);
  if (agent_distance == std::numeric_limits<double>::infinity()) return false;
  for (int i = 0; i < num_robots_; ++i) {
    if (i == agent_id)
      continue;
    else {
      double temp_distance = HeteroDistance(
          hetero_space, scale_factors_, motion_primitives_[i], distance_vec(i),
          grid_location, unreachable_locations_[i]);
      if (temp_distance < agent_distance) return false;
    }
  }
  return true;
}

std::vector<int> Voronoi::GetSingleVoronoiCellIndex(
    const Eigen::MatrixXd &agent_locations, const int &agent_id) {
  Eigen::MatrixXd distance_map = GetDistanceMap(agent_locations);
  std::vector<int> cell_index;
  int count = 0;
  for (int i = 0; i < location_.rows(); ++i) {
    if (IsAgentClosest(hetero_space_, motion_primitives_, distance_map.row(i),
                       location_.row(i), agent_id)) {
      cell_index.push_back(i);
    }
  }
  return cell_index;
}

Eigen::MatrixXd Voronoi::GetSingleVoronoiCell(
    const Eigen::MatrixXd &agent_locations, const int &agent_id) {
  Eigen::MatrixXd distance_map = GetDistanceMap(agent_locations);
  Eigen::MatrixXd cell(location_.rows(), location_.cols());
  int count = 0;
  for (int i = 0; i < location_.rows(); ++i) {
    if (IsAgentClosest(hetero_space_, motion_primitives_, distance_map.row(i),
                       location_.row(i), agent_id)) {
      cell.row(count++) = location_.row(i);
    }
  }
  cell.conservativeResize(count, location_.cols());
  return cell;
}

int Voronoi::FindClosestAgent(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<std::vector<double>> &motion_primitives,
    const Eigen::VectorXd &distance_vec, const Eigen::VectorXd &grid_location) {
  assert(num_robots_ == motion_primitives_.size());
  int closest_agent = -1;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_robots_; ++i) {
    double temp_distance = HeteroDistance(
        hetero_space, scale_factors_, motion_primitives[i], distance_vec(i),
        grid_location, unreachable_locations_[i]);
    if (temp_distance < closest_distance) {
      closest_distance = temp_distance;
      closest_agent = i;
    }
  }
  return closest_agent;
}

std::vector<Eigen::MatrixXd> Voronoi::GetVoronoiCells(
    const Eigen::MatrixXd &agent_locations) {
  Eigen::MatrixXd distance_map = GetDistanceMap(agent_locations);
  Eigen::MatrixXd empty_cell(location_.rows(), location_.cols());
  std::vector<Eigen::MatrixXd> voronoi_cells(num_robots_, empty_cell);
  std::vector<int> sample_counts;
  for (int i = 0; i < location_.rows(); ++i) {
    int closest_agent_id =
        FindClosestAgent(hetero_space_, motion_primitives_, distance_map.row(i),
                         location_.row(i));
    voronoi_cells[i].row(sample_counts[closest_agent_id]++) = location_.row(i);
  }
  for (int i = 0; i < num_robots_; ++i) {
    voronoi_cells[i].conservativeResize(sample_counts[i], location_.cols());
  }
  return voronoi_cells;
}

std::vector<int> Voronoi::GetVoronoiIndex(
    const Eigen::MatrixXd &agent_locations) {
  Eigen::MatrixXd distance_map = GetDistanceMap(agent_locations);
  std::vector<int> cell_labels(location_.rows(), -1);
  for (int i = 0; i < location_.rows(); ++i) {
    cell_labels[i] = FindClosestAgent(hetero_space_, motion_primitives_,
                                      distance_map.row(i), location_.row(i));
  }
  return cell_labels;
}

void Voronoi::UpdateUnreachableLocations(
    const std::vector<Eigen::MatrixXd> &unreachable_locations) {
  assert(unreachable_locations.size() == num_robots_);
  for (int i = 0; i < num_robots_; ++i) {
    for (int j = 0; j < unreachable_locations[i].rows(); ++j) {
      unreachable_locations_[i].insert(
          {unreachable_locations[i](j, 0), unreachable_locations[i](j, 1)});
    }
  }
}

}  // namespace voronoi
}  // namespace sampling