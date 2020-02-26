/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <Eigen/Dense>
#include <vector>

namespace sampling {
enum SAMPLINGMODE { VARIANCE, UCB };
namespace informative_sampling {
class InformativeSampling {
 public:
  InformativeSampling();

  // Normal 2D Voronoi map
  InformativeSampling(const Eigen::MatrixXd &locations,
                      const SAMPLINGMODE &mode, double variance_constant);

  std::pair<double, double> SelectInformativeLocation(
      const Eigen::VectorXd &pred_mean, const Eigen::VectorXd &pred_var,
      const std::vector<int> &cell_index);

 private:
  // THe locations of the entire grid in the map
  Eigen::MatrixXd locations_;

  SAMPLINGMODE mode_;

  double variance_const_;

  double CalculateUtility(const SAMPLINGMODE &mode, const double &mean,
                          const double &var);
};
}  // namespace informative_sampling
}  // namespace sampling