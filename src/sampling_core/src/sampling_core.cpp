#include "sampling_core/sampling_core.h"

#include <std_srvs/Trigger.h>

#include "sampling_msgs/AddTestPositionToModel.h"
#include "sampling_msgs/ModelPredict.h"
#include "sampling_utils/utils.h"

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
      params_(params),
      new_sample_buffer_count_(0) {
  agent_location_subscriber_ =
      nh.subscribe("agent_location_channel", 1,
                   &SamplingCore::AgentLocationUpdateCallback, this);
  sample_subscriber_ = nh.subscribe("sample_channel", 1,
                                    &SamplingCore::SampleUpdateCallback, this);
  modeling_add_sample_client_ =
      nh.serviceClient<sampling_msgs::AddSampleToModel>(KModelingNamespace +
                                                        "add_samples_to_model");
  modeling_update_model_client_ =
      nh.serviceClient<std_srvs::Trigger>(KModelingNamespace + "update_model");

  modeling_predict_client_ = nh.serviceClient<sampling_msgs::ModelPredict>(
      KModelingNamespace + "model_predict");

  sampling_goal_server_ = nh.advertiseService(
      "sampling_goal_channel", &SamplingCore::AssignSamplingGoal, this);
}

bool SamplingCore::Loop() {
  if (new_sample_buffer_count_ >= params_.model_update_frequency_count) {
    if (!UpdateModel()) {
      ROS_WARN_STREAM("Failed to update model!");
      ROS_WARN_STREAM("Retry --- --- ---");
      return false;
    }
    if (!UpdatePrediction()) {
      ROS_WARN_STREAM("Failed to update prediction!");
      ROS_WARN_STREAM("Retry --- --- ---");
      return false;
    }
  }
  return true;
}

void SamplingCore::AgentLocationUpdateCallback(
    const sampling_msgs::AgentLocationConstPtr &msg) {
  // todo: check valid
  agents_locations_[msg->agent_id] = *msg;
}

void SamplingCore::SampleUpdateCallback(
    const sampling_msgs::SampleConstPtr &msg) {
  sample_buffer_.push_back(*msg);
  if (!learning_handler_->UpdateSampleCount(msg->position)) {
    ROS_WARN_STREAM("Failed to update sample account to online learner!");
  }
  return;
}

bool SamplingCore::SampleToSrv(
    const std::vector<sampling_msgs::Sample> &samples,
    sampling_msgs::AddSampleToModel &srv) {
  if (samples.empty()) return false;
  srv.request.positions.reserve(samples.size());
  srv.request.measurements.reserve(samples.size());
  for (const sampling_msgs::Sample &sample : samples) {
    srv.request.positions.push_back(sample.position);
    srv.request.measurements.push_back(sample.data);
  }
  return true;
}

bool InitializeModelAndPrediction() { return false; }

bool SamplingCore::UpdateModel() {
  if (new_sample_buffer_count_ < params_.model_update_frequency_count)
    return false;
  else {
    sampling_msgs::AddSampleToModel add_sample_srv;

    if (!SampleToSrv(sample_buffer_, add_sample_srv)) return false;
    if (modeling_add_sample_client_.call(add_sample_srv) &&
        add_sample_srv.response.success) {
      new_sample_buffer_count_ += sample_buffer_.size();
      sample_buffer_.clear();
    } else {
      return false;
    }

    std_srvs::Trigger update_model_srv;
    if (modeling_update_model_client_.call(update_model_srv) &&
        update_model_srv.response.success) {
      new_sample_buffer_count_ = 0;
      return true;
    } else {
      return false;
    }
  }
}

bool SamplingCore::UpdatePrediction() {
  sampling_msgs::ModelPredict srv;
  if (modeling_predict_client_.call(srv) && srv.response.success) {
    updated_mean_prediction_ = boost::make_optional(srv.response.mean);
    updated_var_prediction_ = boost::make_optional(srv.response.var);
    return true;
  }
  return false;
}

bool SamplingCore::AssignSamplingGoal(
    sampling_msgs::SamplingGoal::Request &req,
    sampling_msgs::SamplingGoal::Response &res) {
  if (!updated_mean_prediction_.is_initialized() ||
      !updated_var_prediction_.is_initialized()) {
    ROS_WARN_STREAM("Unable to assign sampling goal to : "
                    << req.agent_location.agent_id
                    << " due to environment not updated!");
    return false;
  }
  std::vector<sampling_msgs::AgentLocation> agent_locations;
  agent_locations.reserve(params_.agent_ids.size());
  for (const std::string &agent_id : params_.agent_ids) {
    if (!agents_locations_.count(agent_id)) {
      ROS_ERROR_STREAM("Do NOT have location information for " << agent_id);
      return false;
    } else {
      agent_locations.push_back(agents_locations_[agent_id]);
    }
  }
  std::vector<int> partition_index;
  std::vector<int> index_for_map;
  if (!partition_handler_->ComputePartition(req.agent_location.agent_id,
                                            agent_locations, partition_index,
                                            index_for_map)) {
    ROS_ERROR_STREAM("Failed to generate partition for "
                     << req.agent_location.agent_id);
    return false;
  }

  Eigen::MatrixXd agent_responsible_locations;
  if (!utils::ExtractRows(params_.test_locations, partition_index,
                          agent_responsible_locations)) {
    ROS_ERROR_STREAM("Failed to allocate partition locations for "
                     << req.agent_location.agent_id);
    return false;
  }
  std::vector<double> mean =
      utils::Extract(updated_mean_prediction_.get(), partition_index);
  std::vector<double> var =
      utils::Extract(updated_var_prediction_.get(), partition_index);

  geometry_msgs::Point informative_point;

  if (!learning_handler_->InformativeSelection(agent_responsible_locations,
                                               mean, var, informative_point)) {
    ROS_ERROR_STREAM("Failed to select informative point for "
                     << req.agent_location.agent_id);
    return false;
  }
  res.target_position = informative_point;

  return true;
}

}  // namespace core
}  // namespace sampling
