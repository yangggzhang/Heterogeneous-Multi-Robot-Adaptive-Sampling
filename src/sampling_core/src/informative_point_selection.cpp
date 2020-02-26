#include "sampling_core/informative_point_selection.h"

#include <limits>

namespace sampling {
namespace informative_sampling {
InformativeSampling::InformativeSampling() {}

InformativeSampling::InformativeSampling(const Eigen::MatrixXd &locations,
                                         const SAMPLINGMODE &mode,
                                         double variance_constant = 1.0)
    : locations_(locations), mode_(mode), variance_const_(variance_constant) {}

double InformativeSampling::CalculateUtility(const SAMPLINGMODE &mode,
                                             const double &mean,
                                             const double &var) {
  double utility;
  switch (mode) {
    case VARIANCE: {
      utility = var;
      break;
    }
    case UCB: {
      utility = mean + variance_const_ * var;
      break;
    }
    default: {
      utility = 0.0;
      break;
    }
  }
  return utility;
}

std::pair<double, double> InformativeSampling::SelectInformativeLocation(
    const Eigen::VectorXd &pred_mean, const Eigen::VectorXd &pred_var,
    const std::vector<int> &cell_index) {
  std::pair<double, double> informative_location;
  if (cell_index.empty()) return informative_location;
  double most_informative_utility = -std::numeric_limits<double>::infinity();
  int most_informative_index = -1;
  for (const int &index : cell_index) {
    double temp_utility =
        CalculateUtility(mode_, pred_mean(index), pred_var(index));
    if (temp_utility > most_informative_utility) {
      most_informative_utility = temp_utility;
      most_informative_index = index;
    }
  }
  informative_location = {locations_(most_informative_index, 0),
                          locations_(most_informative_index, 1)};
  return informative_location;
}
}  // namespace informative_sampling
}  // namespace sampling