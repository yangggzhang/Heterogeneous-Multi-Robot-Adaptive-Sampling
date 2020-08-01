#include "sampling_core/sampling_core.h"

#include <std_srvs/Trigger.h>

#include "sampling_agent/sampling_agent.h"
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

  // visualization

  std::unique_ptr<visualization::AgentVisualizationHandler>
      agent_visualization_handler = nullptr;

  std::vector<std::unique_ptr<visualization::GridVisualizationHandler>>
      grid_visualization_handlers;

  XmlRpc::XmlRpcValue visualization_param_list;
  if (!ph.getParam("VisualizationProperty", visualization_param_list) ||
      visualization_param_list.size() == 0) {
    ROS_ERROR_STREAM("No visualization setting is provided!");
    return nullptr;
  } else {
    for (int i = 0; i < visualization_param_list.size(); ++i) {
      XmlRpc::XmlRpcValue yaml_node = visualization_param_list[i];
      std::string visualization_type;
      if (!utils::GetParam(yaml_node, "visualization_type",
                           visualization_type)) {
        return nullptr;
      } else {
        if (visualization::KVisualizationType_Location.compare(
                visualization_type) == 0) {
          agent_visualization_handler =
              visualization::AgentVisualizationHandler::MakeUniqueFromXML(
                  nh, yaml_node, int(params.agent_ids.size()),
                  params.test_locations);
        } else if (visualization::KVisualizationType_Grid.compare(
                       visualization_type) == 0 ||
                   visualization::KVisualizationType_Partition.compare(
                       visualization_type) == 0) {
          grid_visualization_handlers.push_back(
              visualization::GridVisualizationHandler::MakeUniqueFromXML(
                  nh, yaml_node, params.test_locations));
          if (grid_visualization_handlers.back() == nullptr) return nullptr;
        } else {
          ROS_ERROR_STREAM("Unkown visualization type");
          return nullptr;
        }
      }
    }
  }

  if (agent_visualization_handler == nullptr) return nullptr;

  std::unique_ptr<SamplingCorePerformanceEvaluation> evaluation_handler;

  if (params.enable_performance_evaluation) {
    evaluation_handler = SamplingCorePerformanceEvaluation::MakeUniqueFromRos(
        nh, params.ground_truth_measurements_vec);
    if (evaluation_handler == nullptr) {
      ROS_ERROR_STREAM("Failed to construct performance evaluator!");
      return nullptr;
    }
  } else {
    evaluation_handler = nullptr;
  }

  return std::unique_ptr<SamplingCore>(new SamplingCore(
      nh, params, std::move(partition_ptr), std::move(learning_ptr),
      std::move(agent_visualization_handler), grid_visualization_handlers,
      std::move(evaluation_handler)));
}

// Constructor

SamplingCore::SamplingCore(
    ros::NodeHandle &nh, const SamplingCoreParams &params,
    std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler,
    std::unique_ptr<learning::OnlineLearningHandler> learning_handler,
    std::unique_ptr<visualization::AgentVisualizationHandler>
        agent_visualization_handler,
    std::vector<std::unique_ptr<visualization::GridVisualizationHandler>>
        &grid_visualization_handlers,
    std::unique_ptr<SamplingCorePerformanceEvaluation> evaluation_handler)
    : params_(params),
      partition_handler_(std::move(partition_handler)),
      learning_handler_(std::move(learning_handler)),
      agent_visualization_handler_(std::move(agent_visualization_handler)),
      evaluation_handler_(std::move(evaluation_handler)),
      is_initialized_(false),
      sample_count_(0) {
  for (int i = 0; i < grid_visualization_handlers.size(); ++i) {
    grid_visualization_handlers_[grid_visualization_handlers[i]->GetName()] =
        std::move(grid_visualization_handlers[i]);
  }

  modeling_add_test_location_client_ =
      nh.serviceClient<sampling_msgs::AddTestPositionToModel>(
          KModelingNamespace + "add_test_position");

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

  kill_agent_server_ =
      nh.advertiseService("kill_agent", &SamplingCore::KillAgent, this);

  for (const std::string &agent_id : params.agent_ids) {
    ros::ServiceClient agent_check_client =
        nh.serviceClient<std_srvs::Trigger>(agent_id + "/check");
    agent_check_clients_.push_back(agent_check_client);
  }
}

bool SamplingCore::Loop() {
  if (!is_initialized_) {
    ROS_INFO_STREAM("Sampling core is starting up!");
    if (!Initialize()) {
      ROS_WARN_STREAM("Failed to initialize sampling core!");
      ROS_WARN_STREAM("Retry --- --- ---");
      return false;
    } else {
      ROS_INFO_STREAM("Sampling core is initialized!");
      is_initialized_ = true;
      return true;
    }
  }

  if (sample_buffer_.size() >= params_.model_update_frequency_count) {
    ROS_INFO_STREAM("Start updating model!");
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

  if (!UpdateVisualization()) {
    ROS_WARN_STREAM("Failed to update visualization!");
    ROS_WARN_STREAM("Retry --- --- ---");
    return false;
  }

  return true;
}

void SamplingCore::AgentLocationUpdateCallback(
    const sampling_msgs::AgentLocationConstPtr &msg) {
  // todo: check valid
  if (!died_agents_.count(msg->agent_id))
    agents_locations_[msg->agent_id] = *msg;
}

void SamplingCore::SampleUpdateCallback(
    const sampling_msgs::SampleConstPtr &msg) {
  ROS_INFO_STREAM("Master received new sample from " << msg->agent_id);
  ROS_INFO_STREAM("Measurement : " << msg->data << " from position ("
                                   << msg->position.x << "," << msg->position.y
                                   << ").");
  sample_count_++;
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

bool SamplingCore::Initialize() {
  for (int i = 0; i < params_.agent_ids.size(); ++i) {
    std_srvs::Trigger srv;
    ros::spinOnce();
    if (!agent_check_clients_[i].call(srv)) {
      ROS_ERROR_STREAM("Failed to connect : " << params_.agent_ids[i]);
      return false;
    }
  }
  if (!InitializeModelAndPrediction()) return false;
  return true;
}

bool SamplingCore::InitializeModelAndPrediction() {
  sampling_msgs::AddTestPositionToModel add_location_srv;
  add_location_srv.request.positions = params_.test_locations_msg;
  if (!modeling_add_test_location_client_.call(add_location_srv) ||
      !add_location_srv.response.success) {
    ROS_ERROR_STREAM("Failed to add test locations to modeling node!");
    return false;
  }

  sampling_msgs::AddSampleToModel add_sample_srv;
  add_sample_srv.request.positions.reserve(params_.initial_measurements.size());
  add_sample_srv.request.measurements.reserve(
      params_.initial_measurements.size());
  for (int i = 0; i < params_.initial_measurements.size(); ++i) {
    add_sample_srv.request.measurements.push_back(
        params_.initial_measurements(i));
    geometry_msgs::Point position;
    position.x = params_.initial_locations(i, 0);
    position.y = params_.initial_locations(i, 1);
    add_sample_srv.request.positions.push_back(position);
  }

  if (modeling_add_sample_client_.call(add_sample_srv) &&
      add_sample_srv.response.success) {
    sample_buffer_.clear();
  } else {
    ROS_ERROR_STREAM("Model add initial samples failed!");
    return false;
  }

  std_srvs::Trigger update_model_srv;
  if (!modeling_update_model_client_.call(update_model_srv) ||
      !update_model_srv.response.success) {
    ROS_ERROR_STREAM("Model initial update failed!");
    return false;
  }

  sampling_msgs::ModelPredict predict_srv;
  if (modeling_predict_client_.call(predict_srv) &&
      predict_srv.response.success) {
    updated_mean_prediction_ = predict_srv.response.mean;
    updated_var_prediction_ = predict_srv.response.var;
  } else {
    ROS_ERROR_STREAM("Model initial prediction failed!");
    return false;
  }
  return true;
}

bool SamplingCore::UpdateModel() {
  if (sample_buffer_.size() < params_.model_update_frequency_count)
    return false;
  else {
    sampling_msgs::AddSampleToModel add_sample_srv;

    if (!SampleToSrv(sample_buffer_, add_sample_srv)) return false;
    if (!modeling_add_sample_client_.call(add_sample_srv) ||
        !add_sample_srv.response.success) {
      return false;
    }

    std_srvs::Trigger update_model_srv;
    sample_buffer_.clear();
    if (modeling_update_model_client_.call(update_model_srv) &&
        update_model_srv.response.success) {
      ROS_INFO_STREAM("Model is updated!");
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool SamplingCore::UpdatePrediction() {
  sampling_msgs::ModelPredict srv;
  if (modeling_predict_client_.call(srv) && srv.response.success) {
    updated_mean_prediction_ = srv.response.mean;
    updated_var_prediction_ = srv.response.var;
    if (evaluation_handler_ != nullptr) {
      if (!evaluation_handler_->UpdatePerformance(
              sample_count_, updated_mean_prediction_, updated_var_prediction_))
        ROS_WARN_STREAM("Failed to update performance evaluation!");
    }
    return true;
  }
  return false;
}

bool SamplingCore::UpdateVisualization() {
  // Update Agent Location
  std::vector<sampling_msgs::AgentLocation> agent_locations_msg;
  agent_locations_msg.reserve(params_.agent_ids.size());
  for (const std::string &agent_id : params_.agent_ids) {
    if (died_agents_.count(agent_id)) {
      sampling_msgs::AgentLocation msg;
      msg.agent_id = agent_id;
      msg.position.x = agent::KRetreatPositionX_m;
      msg.position.y = agent::KRetreatPositionY_m;
      agent_locations_msg.push_back(msg);
    } else if (agents_locations_.count(agent_id))
      agent_locations_msg.push_back(agents_locations_[agent_id]);
  }
  if (!agent_visualization_handler_->UpdateMarker(agent_locations_msg)) {
    ROS_ERROR_STREAM("Failed to update robot location visualization");
    return false;
  }

  for (std::unordered_map<
           std::string,
           std::unique_ptr<visualization::GridVisualizationHandler>>::iterator
           it = grid_visualization_handlers_.begin();
       it != grid_visualization_handlers_.end(); ++it) {
    if (visualization::KPredictionMeanMapName.compare(it->first) == 0) {
      if (updated_mean_prediction_.empty()) {
        ROS_ERROR_STREAM(
            "Prediction Mean for visualization update is not ready yet!");
        return false;
      }
      it->second->UpdateMarker(updated_mean_prediction_);
    } else if (visualization::KPredictionVarianceMapName.compare(it->first) ==
               0) {
      if (updated_var_prediction_.empty()) {
        ROS_ERROR_STREAM(
            "Prediction Variance for visualization update is not ready yet!");
        return false;
      }
      it->second->UpdateMarker(updated_var_prediction_);
    } else if (visualization::KPartitionMapName.compare(it->first) == 0) {
      std::vector<sampling_msgs::AgentLocation> agent_locations;
      agent_locations.reserve(params_.agent_ids.size());
      for (const std::string &agent_id : params_.agent_ids) {
        if (died_agents_.count(agent_id)) {
          continue;
        } else if (!agents_locations_.count(agent_id)) {
          ROS_ERROR_STREAM("Do NOT have location information for " << agent_id);
          return false;
        } else {
          agent_locations.push_back(agents_locations_[agent_id]);
        }
      }

      std::vector<int> partition_index;
      if (!partition_handler_->ComputePartitionForMap(agent_locations,
                                                      partition_index)) {
        ROS_ERROR_STREAM("Failed to generate map partition for visualization!");
        return false;
      } else {
        if (!it->second->UpdateMarker(partition_index)) {
          ROS_ERROR_STREAM("Failed to update map partition for visualization!");
          return false;
        }
      }
    } else {
      ROS_ERROR_STREAM("Unknown grid map visualization update!");
      return false;
    }
  }

  return true;
}  // namespace core

bool SamplingCore::AssignSamplingGoal(
    sampling_msgs::SamplingGoal::Request &req,
    sampling_msgs::SamplingGoal::Response &res) {
  if (!is_initialized_ || updated_mean_prediction_.empty() ||
      updated_var_prediction_.empty()) {
    ROS_WARN_STREAM("Unable to assign sampling goal to : "
                    << req.agent_location.agent_id
                    << " due to environment not updated!");
    return false;
  }

  std::vector<sampling_msgs::AgentLocation> agent_locations;
  agent_locations.reserve(params_.agent_ids.size());
  for (const std::string &agent_id : params_.agent_ids) {
    if (died_agents_.count(agent_id)) {
      continue;
    } else if (!agents_locations_.count(agent_id)) {
      ROS_ERROR_STREAM("Do NOT have location information for " << agent_id);
      return false;
    } else {
      agent_locations.push_back(agents_locations_[agent_id]);
    }
  }

  std::vector<int> partition_index;
  if (!partition_handler_->ComputePartitionForAgent(
          req.agent_location.agent_id, agent_locations, partition_index)) {
    ROS_ERROR_STREAM("Failed to generate partition for "
                     << req.agent_location.agent_id);
    return false;
  }
  if (partition_index.empty()) {
    ROS_WARN_STREAM("Agent : " << req.agent_location.agent_id
                               << " does NOT belong to any partition");
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
      utils::Extract(updated_mean_prediction_, partition_index);
  std::vector<double> var =
      utils::Extract(updated_var_prediction_, partition_index);

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

bool SamplingCore::KillAgent(sampling_msgs::KillAgent::Request &req,
                             sampling_msgs::KillAgent::Response &res) {
  died_agents_.insert(req.agent_id);
  agents_locations_.erase(req.agent_id);
  res.success = true;
  return true;
}

}  // namespace core
}  // namespace sampling
