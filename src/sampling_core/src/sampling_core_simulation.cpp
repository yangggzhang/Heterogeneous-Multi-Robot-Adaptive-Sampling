#include <ros/ros.h>

#include "sampling_core/sampling_core_simulation.h"
#include "sampling_core/utils.h"
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

  initial_sample_size_ = collected_measurements_.rows();

  // Ros service and channel initialization.
  interest_point_assignment_server_ =
      nh_.advertiseService("interest_point_service_channel",
                           &SamplingCoreSimulation::AssignInterestPoint, this);

  agent_location_sub_ =
      nh_.subscribe("agent_location_channel", 10,
                    &SamplingCoreSimulation::AgentLocationCallback, this);

  sample_sub_ =
      nh_.subscribe("sample_collection_channel", 10,
                    &SamplingCoreSimulation::CollectSampleCallback, this);

  visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "sampling_visualization", 1);

  report_pub_ = nh_.advertise<sampling_msgs::report>("report", 1);

  event_timer_ = nh_.createTimer(ros::Duration(1.0),
                                 &SamplingCoreSimulation::ReportCallback, this);

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
  voronoi_node_ = std::unique_ptr<voronoi::Voronoi>(
      new voronoi::Voronoi(test_location_, num_agents_, hetero_spaces_,
                           hetero_scale_factors_, motion_primitives_));

  voronoi_visualization_node_ =
      std::unique_ptr<visualization::VoronoiVisualization>(
          new visualization::VoronoiVisualization(voronoi_visualization_params_,
                                                  test_location_));

  visualization_node_ = std::unique_ptr<visualization::SamplingVisualization>(
      new visualization::SamplingVisualization(graph_visualization_params_,
                                               robot_visualization_params_,
                                               num_agents_, test_location_));

  voronoi_node_->UpdateUnreachableLocations(obstacles_for_robots_);

  UpdateModel();

  ROS_INFO_STREAM("Finish initialization!");
  return true;
}

void SamplingCoreSimulation::ReportCallback(const ros::TimerEvent &) {
  sampling_msgs::report msg;
  msg.header.stamp = ros::Time::now();
  msg.num_samples = collected_measurements_.rows() - initial_sample_size_;
  for (int i = 0; i < agent_locations_.rows(); ++i) {
    msg.location_x.push_back(agent_locations_(i, 0));
    msg.location_y.push_back(agent_locations_(i, 1));
  }
  Eigen::VectorXd gt_pred_mean, gt_pred_var;
  model_->Predict(gt_locations_, gt_pred_mean, gt_pred_var);
  msg.rms = utils::CalculateRMS(gt_pred_mean, gt_measurements_);
  report_pub_.publish(msg);
}

//Coordinate Rotation and Translation
Eigen::MatrixXd SamplingCoreSimulation::CoordinateTransform(double latitude_in, double longitude_in) {
  Eigen::MatrixXd local_gps_matrix = Eigen::MatrixXd::Random(1,2);
  local_gps_matrix(0, 0) = latitude_in;
  local_gps_matrix(0, 1) = longitude_in;
  assert(traj_rotate_matrix_.rows() == 2);
  assert(traj_rotate_matrix_.cols() == 2);
  Eigen::MatrixXd calibrated_gps_matrix = local_gps_matrix * traj_rotate_matrix_;
  calibrated_gps_matrix(0) += traj_lat_offset_;
  calibrated_gps_matrix(1) += traj_lng_offset_;
  return calibrated_gps_matrix;
}

//Coordinate INVERSE Rotation and Translation
Eigen::MatrixXd SamplingCoreSimulation::CoordinateInverseTransform(double latitude_in, double longitude_in) {
  Eigen::MatrixXd local_gps_matrix = Eigen::MatrixXd::Random(1, 2);
  local_gps_matrix(0, 0) = latitude_in - traj_lat_offset_;
  local_gps_matrix(0, 1) = longitude_in - traj_lng_offset_;
  assert(traj_rotate_matrix_.rows() == 2);
  assert(traj_rotate_matrix_.cols() == 2);
  Eigen::MatrixXd calibrated_gps_matrix = local_gps_matrix * traj_rotate_matrix_.inverse();
  return calibrated_gps_matrix;
}

bool SamplingCoreSimulation::AssignInterestPoint(
    sampling_msgs::RequestGoal::Request &req,
    sampling_msgs::RequestGoal::Response &res) {
  ROS_INFO_STREAM(
      "Master Computer received request from robot : " << req.robot_id);
  // Update Voronoi map
  int agent_id = req.robot_id;

  //USE ROTATION TRANSFORM
  Eigen::MatrixXd calibrated_input_gps_matrix = CoordinateTransform(req.robot_latitude, req.robot_longitude);
  agent_locations_(agent_id, 0) = calibrated_input_gps_matrix(0);
  agent_locations_(agent_id, 1) = calibrated_input_gps_matrix(1);


  // agent_locations_(agent_id, 0) = req.robot_latitude;
  // agent_locations_(agent_id, 1) = req.robot_longitude;

  std::vector<int> cell_index =
      voronoi_node_->GetSingleVoronoiCellIndex(agent_locations_, agent_id);

  std::pair<double, double> next_location =
      informative_sampling_node_->SelectInformativeLocation(
          mean_prediction_, var_prediction_, cell_index);

  if ((traj_follow_) && (agent_id ==0)) {
    res.latitude = traj_0_(traj_count_0_,0);
    res.longitude = traj_0_(traj_count_0_,1);
    ++traj_count_0_;
  }
  else if ((traj_follow_)&& (agent_id==1)) {
    res.latitude = traj_1_(traj_count_1_,0);
    res.longitude = traj_1_(traj_count_1_,1); 
    ++traj_count_1_;
  }
  else{
    res.latitude = next_location.first;
    res.longitude = next_location.second;
  }

  //Coordinate INVERSE Rotation and Translation
  Eigen::MatrixXd calibrated_gps_matrix = CoordinateInverseTransform(res.latitude, res.longitude);
  // Eigen::MatrixXd calibrated_gps_matrix = CoordinateTransform(calibrated_gps_matrix_1(0), calibrated_gps_matrix_1(1));
  res.latitude = calibrated_gps_matrix(0);
  res.longitude = calibrated_gps_matrix(1);

  return true;
}

void SamplingCoreSimulation::CollectSampleCallback(
    const sampling_msgs::measurement &msg) {
  if (msg.valid) {
    ROS_INFO_STREAM("Master received temperature : " << msg.measurement);
    update_flag_ = true;

    Eigen::MatrixXd calibrated_input_gps_matrix = CoordinateTransform(msg.location_x, msg.location_y);

    collected_measurements_.conservativeResize(collected_measurements_.size() +
                                               1);
    
    double x = calibrated_input_gps_matrix(0), y = calibrated_input_gps_matrix(1);

    collected_measurements_(collected_measurements_.size() - 1) = poly_coeff_[0] + poly_coeff_[1] * x + poly_coeff_[2] * y +
          poly_coeff_[3] * pow(x, 2) + poly_coeff_[4] * x * y +
          poly_coeff_[5] * pow(y, 2) + poly_coeff_[6] * pow(x, 3) +
          poly_coeff_[7] * pow(x, 2) * y + poly_coeff_[8] * x * pow(y, 2) +
          poly_coeff_[9] * pow(y, 3) + poly_coeff_[10] * pow(x, 4) +
          poly_coeff_[11] * pow(x, 3) * y +
          poly_coeff_[12] * pow(x, 2) * pow(y, 2) +
          poly_coeff_[13] * x * pow(y, 3) + poly_coeff_[14] * pow(y, 4) +
          poly_coeff_[15] * pow(x, 5) + poly_coeff_[16] * pow(x, 4) * y +
          poly_coeff_[17] * pow(x, 3) * pow(y, 2) +
          poly_coeff_[18] * pow(x, 2) * pow(y, 3) +
          poly_coeff_[19] * x * pow(y, 4) + poly_coeff_[20] * pow(y, 5);

    collected_measurements_(collected_measurements_.size() - 1) = std::max(collected_measurements_(collected_measurements_.size() - 1)*1.5,0.0);

    // collected_measurements_(collected_measurements_.size() - 1) =
    //     msg.measurement;

    collected_locations_.conservativeResize(collected_locations_.rows() + 1,
                                            collected_locations_.cols());

    collected_locations_(collected_locations_.rows() - 1, 0) = x; 
    collected_locations_(collected_locations_.rows() - 1, 1) = y;

    // collected_locations_(collected_locations_.rows() - 1, 0) = msg.location_x;
    // collected_locations_(collected_locations_.rows() - 1, 1) = msg.location_y;
  } else {
    ROS_INFO_STREAM(
        "Master computer received invalid sample from : " << msg.robot_id);
  }
}

void SamplingCoreSimulation::AgentLocationCallback(
    const sampling_msgs::agent_location &msg) {
  ROS_INFO_STREAM("Master received location from agent : " << msg.agent_id);

  //USE ROTATION TRANSFORM
  Eigen::MatrixXd calibrated_input_gps_matrix = CoordinateTransform(msg.location_x, msg.location_y);
  agent_locations_(msg.agent_id, 0) = calibrated_input_gps_matrix(0);
  agent_locations_(msg.agent_id, 1) = calibrated_input_gps_matrix(1);

  // agent_locations_(msg.agent_id, 0) = msg.location_x;
  // agent_locations_(msg.agent_id, 1) = msg.location_y;
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

  // learning data
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
    if (!utils::GetParamData(data_path, "test_location", test_location_)) {
      return false;
    }

    if (!utils::GetParamDataVec(data_path, "initial_measurements",
                                collected_measurements_)) {
      return false;
    }

    if (!utils::GetParamData(data_path, "initial_locations",
                             collected_locations_)) {
      return false;
    }

    if (!utils::GetParamData(data_path, "gt_locations", gt_locations_)) {
      return false;
    }

    if (!utils::GetParamDataVec(data_path, "gt_measurements",
                                gt_measurements_)) {
      return false;
    }

    // load trajectory data
    if (!utils::GetParam(data_path, "traj_follow", traj_follow_)) {
      ROS_ERROR_STREAM("Missing traj_follow");
      return false;
    }

    if (traj_follow_){
      if (!utils::GetParamData(data_path, "traj_0", traj_0_)) {
        ROS_ERROR_STREAM("Missing traj_0");
        return false;
      }
      if (!utils::GetParamData(data_path, "traj_1", traj_1_)) {
        ROS_ERROR_STREAM("Missing traj_1_");
        return false;
      }
    }

    if (!utils::GetParam(data_path, "traj_rotate_angle", traj_rotate_angle_)) {
      ROS_ERROR_STREAM("Missing traj_rotate_angle_");
      return false;
    }
    else {
      traj_rotate_matrix_ = Eigen::MatrixXd::Random(2, 2);
      traj_rotate_matrix_(0, 0) = cos(traj_rotate_angle_);
      traj_rotate_matrix_(0, 1) = -sin(traj_rotate_angle_);
      traj_rotate_matrix_(1, 0) = sin(traj_rotate_angle_);
      traj_rotate_matrix_(1, 1) = cos(traj_rotate_angle_);
    }

    if (!utils::GetParam(data_path, "traj_lat_offset", traj_lat_offset_)) {
      ROS_ERROR_STREAM("Missing traj_lat_offset");
      return false;
    }

    if (!utils::GetParam(data_path, "traj_lng_offset", traj_lng_offset_)) {
      ROS_ERROR_STREAM("Missing traj_lng_offset");
      return false;
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
    if (!utils::GetParam(sampling_param, "variance_coef", variance_coef_)) {
      return false;
    }
    ROS_INFO_STREAM("Successfully loaded sampling parameters!");
  }

  // load visualization params
  graph_visualization_params_.clear();
  XmlRpc::XmlRpcValue visualization_param_list;
  ph_.getParam("graph_visualization_parameters", visualization_param_list);
  for (int32_t i = 0; i < visualization_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue visualization_param = visualization_param_list[i];
    visualization::MAP_PARAM param;
    if (!utils::LoadMapParam(visualization_param, param)) {
      ROS_ERROR_STREAM("ERROR LOADING GRAPH VISUALIZATION PARAM!");
      return false;
    }
    graph_visualization_params_.push_back(param);
  }

  ph_.getParam("robot_visualization_parameter", visualization_param_list);
  XmlRpc::XmlRpcValue visualization_param = visualization_param_list[0];
  if (!utils::LoadMapParam(visualization_param, robot_visualization_params_)) {
    ROS_ERROR_STREAM("ERROR LOADING ROBOT VISUALIZATION PARAM!");
    return false;
  }

  ph_.getParam("voronoi_visualization_parameter", visualization_param_list);
  visualization_param = visualization_param_list[0];
  if (!utils::LoadMapParam(visualization_param,
                           voronoi_visualization_params_)) {
    ROS_ERROR_STREAM("ERROR LOADING ROBOT VISUALIZATION PARAM!");
    return false;
  }

  // heteregeneous parameter
  XmlRpc::XmlRpcValue heterogeneous_param_list;
  ph_.getParam("heterogeneous_parameter", heterogeneous_param_list);
  XmlRpc::XmlRpcValue heterogeneous_param = heterogeneous_param_list[0];
  std::vector<int> hetero_int;
  if (!utils::GetParam(heterogeneous_param, "hetero_spaces", hetero_int)) {
    ROS_ERROR_STREAM("ERROR LOADING ROBOT HETERO PARAM!");
    return false;
  }

  hetero_spaces_.clear();
  for (const int &index : hetero_int) {
    switch (index) {
      case 0: {
        hetero_spaces_.push_back(HeterogenitySpace::DISTANCE);
        break;
      }
      case 1: {
        hetero_spaces_.push_back(HeterogenitySpace::SPEED);
        break;
      }
      case 2: {
        hetero_spaces_.push_back(HeterogenitySpace::BATTERYLIFE);
        break;
      }
      case 3: {
        hetero_spaces_.push_back(HeterogenitySpace::MOBILITY);
        break;
      }
      case 4: {
        hetero_spaces_.push_back(HeterogenitySpace::REACHABILITY);
        break;
      }
      default: {
        ROS_ERROR_STREAM("Undefined heterogeneous space!");
        return -1;
      }
    }
  }

  if (!utils::GetParam(heterogeneous_param, "hetero_scale_factors",
                       hetero_scale_factors_)) {
    ROS_ERROR_STREAM("ERROR LOADING ROBOT HETERO PARAM!");
    return false;
  }

  if (!utils::GetParam(heterogeneous_param, "poly_coeff",
                       poly_coeff_)) {
    ROS_ERROR_STREAM("ERROR LOADING ROBOT poly_coeff PARAM!");
    return false;
  }

  motion_primitives_.resize(num_agents_);
  for (int i = 0; i < num_agents_; ++i) {
    std::string param_name = "mp" + std::to_string(i);
    if (!utils::GetParam(heterogeneous_param, param_name,
                         motion_primitives_[i])) {
      ROS_ERROR_STREAM("ERROR LOADING ROBOT HETERO PARAM!");
      return false;
    }
  }

  obstacles_for_robots_.resize(num_agents_);

  XmlRpc::XmlRpcValue obstacle_list;
  if (!ph_.getParam("obstacle_list", obstacle_list)) {
    ROS_ERROR_STREAM("Missing model parameters");
    return false;
  } else {
    if (obstacle_list.size() == 0) {
      ROS_ERROR_STREAM("Empty model parameters!");
      return false;
    }
    for (int32_t i = 0; i < obstacle_list.size(); ++i) {
      XmlRpc::XmlRpcValue obstacle_param = obstacle_list[i];
      int agent_id;
      if (!sampling::utils::GetParam(obstacle_param, "agent_id", agent_id)) {
        // return false;
      }
      if (!sampling::utils::GetParamData(obstacle_param, "obstacle_path",
                                         obstacles_for_robots_[agent_id])) {
        // return false;
      }
    }
    ROS_INFO_STREAM("Successfully loaded model parameters!");
  }

  ROS_INFO_STREAM("Finish loading data!");
  return true;
}

void SamplingCoreSimulation::UpdateModel() {
  model_->Train(collected_measurements_, collected_locations_);
  model_->Predict(test_location_, mean_prediction_, var_prediction_);
}

void SamplingCoreSimulation::UpdateVisualization(const bool &update_model) {
  visualization_msgs::MarkerArray marker_array;
  if (update_model) {
    marker_array =
    // Change To Update GT
        visualization_node_->UpdateMap({mean_prediction_, gt_measurements_});
    // Visualize Mean and Variance
        // visualization_node_->UpdateMap({mean_prediction_, var_prediction_});
  }
  marker_array.markers.push_back(
      visualization_node_->UpdateRobot(agent_locations_));
  std::vector<int> cell_index =
      voronoi_node_->GetVoronoiIndex(agent_locations_);
  voronoi_visualization_node_->UpdateMap(cell_index);
  marker_array.markers.push_back(voronoi_visualization_node_->GetVoronoiMap());
  visualization_pub_.publish(marker_array);
}

void SamplingCoreSimulation::Update() {
  UpdateVisualization(update_flag_);
  if (update_flag_) {
    ROS_INFO_STREAM("Update!");
    UpdateModel();
    // ROS_INFO_STREAM("Update MODEL!");
    UpdateVisualization(update_flag_);
    // ROS_INFO_STREAM("Update VISUALIZATION!");
    update_flag_ = false;
  }
}

}  // namespace core
}  // namespace sampling
