/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang (yangzha4@andrew.cmu.edu)
 */

#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "sampling_core/voronoi.h"

namespace sampling {
enum SAMPLINGMODE { VARIANCE, UCB };
namespace informative_sampling {
class InformativeSampling {
 public:
  InformativeSampling();

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

  double CalculateUtility(const SAMPLINGMODE &mode, const int &location_id,
                          const double &mean, const double &var);

  std::unordered_map<int, int> collection_count_;
};
}  // namespace informative_sampling
}  // namespace sampling