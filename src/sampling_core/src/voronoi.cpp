#include "sampling_core/voronoi.h"
#include <ros/ros.h>

namespace sampling {
namespace voronoi {

Voronoi::Voronoi() {}

Voronoi::Voronoi(const Eigen::MatrixXd &location) : location_(location) {}

bool Voronoi::UpdateVoronoiMap(const Eigen::MatrixXd &agent_locations,
                               const Eigen::VectorXd &scale_factor,
                               Eigen::VectorXi &labels,
                               Eigen::MatrixXd &distance_matrix) {
  if (location_.rows() == 0 || agent_locations.rows() != scale_factor.rows()) {
    ROS_ERROR("Can not construct voronoi map!");
    return false;
  }

  labels.resize(location_.rows());
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
    labels(i) = distance.minCoeff();
  }
  return true;
}

Eigen::MatrixXd Voronoi::GetLocation() { return location_; }

}  // namespace voronoi
}  // namespace sampling