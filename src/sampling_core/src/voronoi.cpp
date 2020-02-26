#include "sampling_core/voronoi.h"

#include <math.h> /* sqrt */
#include <ros/ros.h>

namespace sampling {
namespace voronoi {

Voronoi::Voronoi() {}

Voronoi::Voronoi(const Eigen::MatrixXd &location) : location_(location) {}

Voronoi::Voronoi(const Eigen::MatrixXd &location,
                 const std::vector<std::vector<double>> &motion_primitives)
    : location_(location), motion_primitives_(motion_primitives) {}

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

double Voronoi::L1_Distance(const std::vector<double> &lhs,
                            const std::vector<double> &rhs) {
  assert(lhs.size() == rhs.size());
  double distance = 0.0;
  for (int i = 0; i < lhs.size(); ++i) {
    distance += fabs(lhs[i] - rhs[i]);
  }
  return distance;
}

double Voronoi::L2_Distance(const std::vector<double> &lhs,
                            const std::vector<double> &rhs) {
  assert(lhs.size() == rhs.size());
  double distance = 0.0;
  for (int i = 0; i < lhs.size(); ++i) {
    distance += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
  }
  return std::sqrt(distance);
}

inline double Voronoi::Continuous_Distance(const double &motion_primitive,
                                           const double &euclidean_distance) {
  double px = exp(motion_primitive * euclidean_distance);
  double nx = exp(-motion_primitive * euclidean_distance);
  return (px - nx) / (px + nx);
}

}  // namespace voronoi
}  // namespace sampling