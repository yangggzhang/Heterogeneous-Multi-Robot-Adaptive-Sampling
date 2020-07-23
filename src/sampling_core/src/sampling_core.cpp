#include "sampling_core/sampling_core.h"

#include <sampling_msgs/AddTestPositionToModel.h>
#include <std_srvs/Trigger.h>

namespace sampling {
namespace core {

std::unique_ptr<SamplingCore> SamplingCore::MakeUniqueFromRos(
    ros::NodeHandle &nh, ros::NodeHandle &ph) {
  SamplingCoreParams params;
  if (!params.LoadFromRosParams(ph)) {
    ROS_ERROR_STREAM("Failed to load core parameters for the sampling task!");
    return nullptr;
  }

  for (const std::string &agent_id : params.agent_ids) {
    ros::ServiceClient agent_check_client =
        nh.serviceClient<std_srvs::Trigger>(agent_id + "/check");
    std_srvs::Trigger srv;
    if (!agent_check_client.call(srv)) {
      ROS_ERROR_STREAM("Failed to connect : " << agent_id);
      return nullptr;
    }
  }

  // Modeling
  ros::ServiceClient modeling_add_test_position_client =
      nh.serviceClient<std_srvs::Trigger>(KModelingNamespace +
                                          "add_test_position");
  sampling_msgs::AddTestPositionToModel srv;
  srv.request.positions = params.test_locations_msg;
  if (!modeling_add_test_position_client.call(srv) || !srv.response.success) {
    ROS_ERROR_STREAM("Failed to connect sampling modeling node!");
    return nullptr;
  }

  std::unique_ptr<partition::WeightedVoronoiPartition> partition_ptr =
      partition::WeightedVoronoiPartition::MakeUniqueFromRosParam(
          params.agent_ids, params.test_locations, ph);
  if (partition_ptr == nullptr) {
    ROS_ERROR_STREAM("Failed to create sampling patition handler!");
    return nullptr;
  }

  std::unique_ptr<learning::OnlineLearningHandler> learning_ptr =
      learning::OnlineLearningHandler::MakeUniqueFromRosParam(ph);

  if (learning_ptr == nullptr) {
    ROS_ERROR_STREAM("Failed to create sampling learning handler!");
    return nullptr;
  }

  return std::unique_ptr<SamplingCore>(new SamplingCore(
      nh, std::move(partition_ptr), std::move(learning_ptr), params));
}

SamplingCore::SamplingCore(
    ros::NodeHandle &nh,
    std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler,
    std::unique_ptr<learning::OnlineLearningHandler> learning_handler,
    const SamplingCoreParams &params)
    : partition_handler_(std::move(partition_handler)),
      learning_handler_(std::move(learning_handler)),
      params_(params) {}
}  // namespace core
}  // namespace sampling
