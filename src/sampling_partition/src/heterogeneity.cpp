#include "sampling_partition/heterogeneity.h"

#include <math.h> /* sqrt */

#include "sampling_partition/heterogeneity_distance.h"
#include "sampling_partition/heterogeneity_distance_dependent.h"
#include "sampling_partition/heterogeneity_topography_dependent.h"

namespace sampling {
namespace partition {

Heterogeneity::Heterogeneity(const HeterogeneityParams &params,
                             const Eigen::MatrixXd &map)
    : params_(params), map_(map) {}

std::string Heterogeneity::GetType() { return params_.heterogeneity_type; }

}  // namespace partition
}  // namespace sampling