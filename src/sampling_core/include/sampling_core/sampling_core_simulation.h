#pragma once

#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/optional.hpp>
#include <queue>
#include <string>
#include <unordered_map>

#include "sampling_core/gmm.h"
#include "sampling_core/gpmm.h"
#include "sampling_core/informative_point_selection.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"
#include "sampling_core/voronoi.h"
#include "sampling_core/voronoi_visualization.h"
#include "sampling_msgs/RequestGoal.h"
#include "sampling_msgs/agent_location.h"
#include "sampling_msgs/report.h"

namespace sampling {
namespace core {

class SamplingCoreSimulation {
 public:
  SamplingCoreSimulation() {}

  SamplingCoreSimulation(const ros::NodeHandle &nh, const ros::NodeHandle &ph);

  bool Initialize();

  bool ParseFromRosParam();

  void UpdateModel();

  void UpdateVisualization(const bool &update_model);

  void Update();

  bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode,
                    visualization::MAP_PARAM &param);

 private:
  // ROS
  ros::NodeHandle nh_, ph_;

  ros::Timer event_timer_;

  int num_agents_;

  ros::Subscriber agent_location_sub_;

  ros::Subscriber sample_sub_;

  ros::Publisher visualization_pub_;

  ros::Publisher report_pub_;

  void CollectSampleCallback(const sampling_msgs::measurement &msg);

  void AgentLocationCallback(const sampling_msgs::agent_location &msg);

  bool AssignInterestPoint(sampling_msgs::RequestGoal::Request &req,
                           sampling_msgs::RequestGoal::Response &res);

  void ReportCallback(const ros::TimerEvent &);

  // Agent locations matrix, an num_agent x 2 matrix
  Eigen::MatrixXd agent_locations_;

  // model parameter
  int num_gaussian_;

  int initial_sample_size_;

  std::vector<std::vector<double>> gp_hyperparams_;

  int max_iteration_;

  double eps_;

  bool update_flag_;

  std::unique_ptr<gpmm::GaussianProcessMixtureModel> model_;

  std::unique_ptr<gpmm::GaussianProcessMixtureModel> gt_model_;

  // data
  Eigen::MatrixXd test_location_;

  Eigen::VectorXd gt_measurements_;

  Eigen::MatrixXd gt_locations_;

  Eigen::VectorXd collected_measurements_;

  Eigen::MatrixXd collected_locations_;

  // prediction
  Eigen::VectorXd mean_prediction_;

  Eigen::VectorXd var_prediction_;

  // Voronoi
  std::unique_ptr<voronoi::Voronoi> voronoi_node_;

  // Informative Selection Params
  SAMPLINGMODE selection_mode_;

  double variance_coef_;

  std::unique_ptr<informative_sampling::InformativeSampling>
      informative_sampling_node_;

  ros::ServiceServer interest_point_assignment_server_;

  // visualization
  std::unique_ptr<visualization::SamplingVisualization> visualization_node_;

  std::vector<visualization::MAP_PARAM> graph_visualization_params_;

  visualization::MAP_PARAM robot_visualization_params_;

  std::unique_ptr<visualization::VoronoiVisualization>
      voronoi_visualization_node_;

  visualization::MAP_PARAM voronoi_visualization_params_;

  // heteregeneous
  std::vector<HeterogenitySpace> hetero_spaces_;

  std::vector<double> hetero_scale_factors_;

  std::vector<std::vector<double>> motion_primitives_;

  std::vector<Eigen::MatrixXd> obstacles_for_robots_;
};
}  // namespace core
}  // namespace sampling
