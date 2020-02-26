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
      motion_primitives_(motion_primitives) {}

bool Voronoi::UpdateVoronoiMap(const Eigen::MatrixXd &agent_locations,
                               const Eigen::VectorXd &scale_factor,
                               std::vector<std::vector<int>> &labels,
                               Eigen::MatrixXd &distance_matrix) {
  if (location_.rows() == 0 || agent_locations.rows() != scale_factor.rows()) {
    ROS_ERROR("Can not construct voronoi map!");
    return false;
  }
  labels.clear();
  labels.resize(agent_locations.rows());
  distance_matrix =
      Eigen::MatrixXd::Zero(location_.rows(), agent_locations.rows());

  for (int i = 0; i < location_.rows(); i++) {
    Eigen::MatrixXd distance_mat = agent_locations;
    distance_mat.col(0).array() -= location_(i, 0);
    distance_mat.col(1).array() -= location_(i, 1);
    distance_mat = distance_mat.cwiseProduct(distance_mat);
    Eigen::VectorXd distance =
        distance_mat.col(0).array() + distance_mat.col(1).array();
    distance = distance.array().sqrt();
    distance *= scale_factor;
    distance_matrix.row(i) = distance;
    labels[distance.minCoeff()].push_back(i);
  }
  return true;
}

Eigen::MatrixXd Voronoi::GetLocation() { return location_; }

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
    distance_map.col(i) = distance.rowwise().norm();
    max_distance = std::max(max_distance, distance_map.col(i).maxCoeff());
  }
  return distance_map.array() / max_distance;
}

double Voronoi::HeteroDistance(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<double> &motion_primitive,
    const double &euclidean_distance) {
  Eigen::VectorXd distance_vec(hetero_space.size());
  for (int i = 0; i < hetero_space.size(); ++i) {
    switch (hetero_space[i]) {
      case DISTANCE: {
        // distance_vec(i) = euclidean_distance;
        distance_vec(i) =
            ContinuousDistance(motion_primitive[i], euclidean_distance);
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
      default:
        break;
    }
  }
  return distance_vec.norm();
}

bool Voronoi::IsAgentClosest(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<std::vector<double>> &motion_primitives,
    const Eigen::VectorXd &distance_vec, const int &agent_id) {
  assert(distance_vec.size() == num_robots_);
  const double agent_distance = HeteroDistance(
      hetero_space, motion_primitives_[agent_id], distance_vec(agent_id));
  for (int i = 0; i < num_robots_; ++i) {
    if (i == agent_id)
      continue;
    else {
      double temp_distance =
          HeteroDistance(hetero_space, motion_primitives_[i], distance_vec(i));
      if (temp_distance < agent_distance) return false;
    }
  }
  return true;
}

Eigen::MatrixXd Voronoi::GetSingleVoronoiCell(
    const Eigen::MatrixXd &agent_locations, const int &agent_id) {
  Eigen::MatrixXd distance_map = GetDistanceMap(agent_locations);
  Eigen::MatrixXd cell(location_.rows(), location_.cols());
  int count = 0;
  for (int i = 0; i < location_.rows(); ++i) {
    if (IsAgentClosest(hetero_space_, motion_primitives_, distance_map.row(i),
                       agent_id)) {
      cell.row(count++) = location_.row(i);
    }
  }
  cell.conservativeResize(count, location_.cols());
  return cell;
}

int Voronoi::FindClosestAgent(
    const std::vector<HeterogenitySpace> &hetero_space,
    const std::vector<std::vector<double>> &motion_primitives,
    const Eigen::VectorXd &distance_vec) {
  assert(num_robots_ == motion_primitives_.size());
  int closest_agent = -1;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_robots_; ++i) {
    double temp_distance =
        HeteroDistance(hetero_space, motion_primitives[i], distance_vec(i));
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
    int closest_agent_id = FindClosestAgent(hetero_space_, motion_primitives_,
                                            distance_map.row(i));
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
                                      distance_map.row(i));
  }
  return cell_labels;
}

}  // namespace voronoi
}  // namespace sampling