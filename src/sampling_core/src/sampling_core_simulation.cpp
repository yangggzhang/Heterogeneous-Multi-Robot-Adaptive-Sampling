#include <ros/ros.h>

#include "sampling_core/sampling_core_simulation.h"
#include "sampling_msgs/RequestLocation.h"

#include <visualization_msgs/MarkerArray.h>

namespace sampling {
namespace core {

SamplingCoreSimulation::SamplingCoreSimulation(const ros::NodeHandle &nh,
                                               const ros::NodeHandle &ph)
    : nh_(nh), ph_(ph) {}

bool SamplingCoreSimulation::Initialize() {
  // Load parameters
  if (!ParseFromRosParam()) {
    ROS_ERROR_STREAM("Missing required ros parameter");
    return false;
  }

  // initialize private elements
  agent_locations_ = Eigen::MatrixXd::Zero(num_agents_, 2);

  update_flag_ = false;

  // Ros service and channel initialization.
  interest_point_assignment_server_ =
      nh_.advertiseService("interest_point_service_channel",
                           &SamplingCoreSimulation::AssignInterestPoint, this);

  agent_location_sub_ =
      nh_.subscribe("agent_location_channel", 1,
                    &SamplingCoreSimulation::AgentLocationCallback, this);

  sample_sub_ =
      nh_.subscribe("sample_collection_channel", 1,
                    &SamplingCoreSimulation::CollectSampleCallback, this);

  // Model initialization
  model_ = std::unique_ptr<gpmm::GaussianProcessMixtureModel>(
      new gpmm::GaussianProcessMixtureModel(num_gaussian_, gp_hyperparams_,
                                            max_iteration_, eps_));

  // Information selection initialization
  informative_sampling_node_ =
      std::unique_ptr<informative_sampling::InformativeSampling>(
          new informative_sampling::InformativeSampling(
              test_location_, selection_mode_, variance_coef_));

  // Voronoi Update
  voronoi_node_ =
      std::unique_ptr<voronoi::Voronoi>(new voronoi::Voronoi(test_location_));

  UpdateModel();

  ROS_INFO_STREAM("Finish initialization!");
  return true;
}

bool SamplingCoreSimulation::AssignInterestPoint(
    sampling_msgs::RequestGoal::Request &req,
    sampling_msgs::RequestGoal::Response &res) {
  ROS_INFO_STREAM(
      "Master Computer received request from robot : " << req.robot_id);

  // Update Voronoi map
  int agent_id = req.robot_id;

  std::vector<int> cell_index =
      voronoi_node_->GetSingleVoronoiCellIndex(agent_locations_, agent_id);

  std::pair<double, double> next_location =
      informative_sampling_node_->SelectInformativeLocation(
          mean_prediction_, var_prediction_, cell_index);

  res.latitude = next_location.first;
  res.longitude = next_location.second;

  return true;
}

void SamplingCoreSimulation::CollectSampleCallback(
    const sampling_msgs::measurement &msg) {
  if (msg.valid) {
    ROS_INFO_STREAM("Master received temperature : " << msg.measurement);
    update_flag_ = true;

    collected_measurements_.conservativeResize(collected_measurements_.size() +
                                               1);
    collected_measurements_(collected_measurements_.size() - 1) =
        msg.measurement;

    collected_locations_.conservativeResize(collected_locations_.rows() + 1,
                                            collected_locations_.cols());
    collected_locations_(collected_locations_.rows() - 1, 0) = msg.location_x;
    collected_locations_(collected_locations_.rows() - 1, 1) = msg.location_y;
  } else {
    ROS_INFO_STREAM(
        "Master computer received invalid sample from : " << msg.robot_id);
  }
}

void SamplingCoreSimulation::AgentLocationCallback(
    const sampling_msgs::agent_location &msg) {
  ROS_INFO_STREAM("Master received location from agent : " << msg.agent_id);
  agent_locations_(msg.agent_id, 0) = msg.location_x;
  agent_locations_(msg.agent_id, 1) = msg.location_y;
}

bool SamplingCoreSimulation::ParseFromRosParam() {
  // Number of Agent
  if (!ph_.getParam("num_agents", num_agents_)) {
    ROS_ERROR_STREAM("Missing number of agents!");
    return false;
  }

  // learning Model parameters
  XmlRpc::XmlRpcValue model_param_list;
  if (!ph_.getParam("model_parameters", model_param_list)) {
    ROS_ERROR_STREAM("Missing model parameters");
    return false;
  } else {
    if (model_param_list.size() == 0) {
      ROS_ERROR_STREAM("Empty model parameters!");
      return false;
    }
    XmlRpc::XmlRpcValue model_param = model_param_list[0];
    if (!utils::GetParam(model_param, "num_gaussian", num_gaussian_)) {
      return false;
    }
    for (int i = 0; i < num_gaussian_; ++i) {
      std::string param_name = "param" + std::to_string(i);
      std::vector<double> gp_hyper_param;
      if (!utils::GetParam(model_param, param_name, gp_hyper_param)) {
        return false;
      }
      gp_hyperparams_.push_back(gp_hyper_param);
    }
    if (!utils::GetParam(model_param, "max_iteration", max_iteration_)) {
      return false;
    }
    if (!utils::GetParam(model_param, "eps", eps_)) {
      return false;
    }
    ROS_INFO_STREAM("Successfully loaded model parameters!");
  }

  /// learning data
  XmlRpc::XmlRpcValue data_list;
  if (!ph_.getParam("data_path", data_list)) {
    ROS_ERROR_STREAM("Missing necessary data");
    return false;
  } else {
    if (data_list.size() == 0) {
      ROS_ERROR_STREAM("Empty data path parameters!");
      return false;
    }
    XmlRpc::XmlRpcValue data_path = data_list[0];
    std::string location_data;
    if (!utils::GetParamData(data_path, "test_location", test_location_)) {
      return false;
    }

    if (!utils::GetParamDataVec(data_path, "initial_measurements",
                                collected_measurements_)) {
    }

    if (!utils::GetParamData(data_path, "initial_locations",
                             collected_locations_)) {
    }
    ROS_INFO_STREAM("Successfully loaded data!");
  }

  // learning sampling parameter
  XmlRpc::XmlRpcValue sampling_param_list;
  if (!ph_.getParam("sampling_parameters", sampling_param_list)) {
    ROS_ERROR_STREAM("Missing sampling parameters");
    return false;
  } else {
    if (sampling_param_list.size() == 0) {
      ROS_ERROR_STREAM("Empty sampling parameters!");
      return false;
    }
    XmlRpc::XmlRpcValue sampling_param = sampling_param_list[0];

    int selection_model;
    if (!utils::GetParam(sampling_param, "selection_model", selection_model)) {
      return false;
    } else {
      switch (selection_model) {
        case 0: {
          selection_mode_ = VARIANCE;
          break;
        }
        case 1: {
          selection_mode_ = UCB;
          break;
        }
        default: {
          selection_mode_ = VARIANCE;
          break;
        }
      }
    }

    if (!utils::GetParam(sampling_param, "selection_model", selection_model)) {
      return false;
    }
    if (!utils::GetParam(sampling_param, "variance_coef_", variance_coef_)) {
      return false;
    }
    ROS_INFO_STREAM("Successfully loaded sampling parameters!");
  }
  ROS_INFO_STREAM("Finish loading data!");
  return true;
}

void SamplingCoreSimulation::UpdateModel() {
  model_->Train(collected_measurements_, collected_locations_);
  model_->Predict(test_location_, mean_prediction_, var_prediction_);
}

void SamplingCoreSimulation::Update() {
  if (update_flag_) {
    ROS_INFO_STREAM("Update!");
    update_flag_ = false;
    UpdateModel();
  }
}

}  // namespace core
}  // namespace sampling
