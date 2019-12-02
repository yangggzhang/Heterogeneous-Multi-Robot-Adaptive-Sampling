#pragma once

#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/optional.hpp>
#include <queue>
#include <string>
#include <unordered_map>

#include "sampling_core/gmm_utils.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"
#include "sampling_core/voronoi.h"
#include "sampling_msgs/RequestGoal.h"

namespace sampling {
namespace core {

enum HeuristicMode { VARIANCE, UCB, DISTANCE_UCB };

class GPSHashFunction {
 public:
  double operator()(const Eigen::MatrixXd &GPS) const {
    return (GPS(0, 1) + 180.0) * 180.0 + GPS(0, 0);
  }
};

using pq = std::priority_queue<std::pair<double, int>,
                               std::vector<std::pair<double, int>>,
                               std::less<std::pair<double, int>>>;

class SamplingCore {
 public:
  SamplingCore(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  bool Init();

  bool AssignInterestPoint(sampling_msgs::RequestGoal::Request &req,
                           sampling_msgs::RequestGoal::Response &res);

  void UpdateHeuristic();

  bool ParseFromRosParam();

  bool InitializeVisualization();

  void UpdateGPModel();

  void UpdateVisualization();

  void Update();

  bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode,
                    visualization::MAP_PARAM &param);

  double RMSError(const Eigen::VectorXd &val1, const Eigen::VectorXd &val2);

 private:
  // ROS
  ros::NodeHandle nh_, rh_;
  ros::Publisher distribution_visualization_pub_;
  ros::Subscriber sample_sub_;
  ros::Subscriber Jackal_GPS_sub_;
  ros::Subscriber Pelican_GPS_sub_;

  Eigen::VectorXd gt_mean_, gt_var_;

  void CollectSampleCallback(const sampling_msgs::measurement &msg);

  void JackalGPSCallback(const sensor_msgs::NavSatFix &msg);

  void PelicanGPSCallback(const sensor_msgs::NavSatFix &msg);

  boost::optional<double> Jackal_latitude_, Jackal_longitude_;
  boost::optional<double> Pelican_latitude_, Pelican_longitude_;

  // interest point assignment

  // gp parameter
  int gp_num_gaussian_;
  std::vector<double> gp_hyperparam_;
  double map_scale_;

  // EM parameter
  double convergence_threshold_;
  int max_iteration_;
  int model_update_rate_;

  // data
  Eigen::MatrixXd location_;
  Eigen::MatrixXd ground_truth_location_;
  Eigen::MatrixXd ground_truth_temperature_;
  Eigen::MatrixXd init_sample_location_;
  Eigen::MatrixXd init_sample_temperature_;

  // prediction
  Eigen::VectorXd mean_prediction_;
  Eigen::VectorXd var_prediction_;

  // GP parameter
  int ground_truth_num_gaussian_;
  int num_gaussian_;
  std::vector<double> gp_hyperparameter_;
  gmm::Gaussian_Mixture_Model gp_node_;
  gmm::Gaussian_Mixture_Model gt_gp_node_;

  // Visualization
  std::unordered_map<std::string,
                     std::unique_ptr<visualization::SamplingVisualization>>
      visualization_node_;

  std::unique_ptr<visualization::RobotVisualization> Jackal_visualization_node_;
  std::unique_ptr<visualization::RobotVisualization>
      Pelican_visualization_node_;

  std::vector<visualization::MAP_PARAM> visualization_params_;

  // sampling
  bool update_flag_;
  int sample_size_;
  voronoi::Voronoi voronoi_cell_;
  std::string Jackal_id_;
  std::string Pelican_id_;
  ros::ServiceClient Jackal_GPS_client_;
  ros::ServiceClient Pelican_GPS_client_;
  Eigen::VectorXd distance_scale_factor_;
  double variance_coeff_;
  std::unordered_map<Eigen::MatrixXd, double, GPSHashFunction> sample_count_;
  pq heuristic_pq_;
  std::vector<pq> heuristic_pq_v_;
  ros::ServiceServer interest_point_assignment_ser_;
  HeuristicMode heuristic_mode_;
};
}  // namespace core
}  // namespace sampling
