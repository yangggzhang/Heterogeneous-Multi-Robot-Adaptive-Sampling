#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <algorithm>  // std::min_element, std::max_element
#include <queue>
#include <string>
#include <unordered_map>

#include "sampling_core/gmm_utils.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"
#include "sampling_core/voronoi.h"
#include "sampling_msgs/RequestGoal.h"
#include "sampling_msgs/RequestLocation.h"

namespace sampling {

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
class CentralizedSamplingNode {
 public:
  CentralizedSamplingNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    ROS_INFO_STREAM("start node!");

    // Load parameters
    if (!ParseFromRosParam()) {
      ROS_ERROR_STREAM("Missing required ros parameter");
    }

    // Initialize visualization
    if (!InitializeVisualization()) {
      ROS_ERROR_STREAM("Failed to initialize visualization");
    }

    //
    interest_point_assignment_ser_ = nh_.advertiseService(
        "interest_point_service_channel",
        &CentralizedSamplingNode::assign_interest_point, this);

    // Sampling
    sample_sub_ =
        nh_.subscribe("temperature_update_channel", 1,
                      &CentralizedSamplingNode::collect_sample_callback, this);

    voronoi_cell_ = voronoi::Voronoi(location_);
    update_flag_ = false;
    sample_size_ = 0;

    // GP
    gp_node_ = gmm::Gaussian_Mixture_Model(num_gaussian_, gp_hyperparameter_);
    gt_gp_node_ = gmm::Gaussian_Mixture_Model(ground_truth_num_gaussian_,
                                              gp_hyperparameter_);

    // Robot agent
    Jackal_GPS_client_ = nh_.serviceClient<sampling_msgs::RequestLocation>(
        "Jackal_request_GPS_channel");

    Pelican_GPS_client_ = nh_.serviceClient<sampling_msgs::RequestLocation>(
        "Pelican_request_GPS_channel");

    // Initial data
    if (init_sample_temperature_.rows() > 0) {
      gp_node_.add_training_data(init_sample_location_,
                                 init_sample_temperature_);
      update_gp_model();
      update_heuristic();
      ROS_INFO_STREAM("Initialize GP model with initial data points");
    }

    if (ground_truth_temperature_.rows() > 0) {
      gt_gp_node_.add_training_data(location_, ground_truth_temperature_);
      gt_gp_node_.expectation_maximization(max_iteration_,
                                           convergence_threshold_);
      gt_gp_node_.GaussianProcessMixture_predict(location_, gt_mean_, gt_var_);
      visualization_node_["gt"]->update_map(gt_mean_);
      visualization_node_["raw"]->update_map(init_sample_temperature_.col(0));
    }
    // test
  }

  bool assign_interest_point(sampling_msgs::RequestGoal::Request &req,
                             sampling_msgs::RequestGoal::Response &res) {
    ROS_INFO_STREAM(
        "Master Computer received request from robot : " << req.robot_id);
    // todo \yang add hetegeneous functionality
    switch (heuristic_mode_) {
      case VARIANCE: {
        if (heuristic_pq_.empty()) {
          gp_node_.GaussianProcessMixture_predict(location_, mean_prediction_,
                                                  var_prediction_);
          update_heuristic();
          if (heuristic_pq_.empty()) {
            ROS_ERROR_STREAM("Error! Heuristice priority queue empty!");
            return false;
          }
        }
        std::pair<double, int> interest_point_pair = heuristic_pq_.top();
        res.latitude = location_(interest_point_pair.second, 0) / gp_scale_;
        res.longitude = location_(interest_point_pair.second, 1) / gp_scale_;
        heuristic_pq_.pop();
        return true;
      }
      case UCB: {
        if (heuristic_pq_.empty()) {
          gp_node_.GaussianProcessMixture_predict(location_, mean_prediction_,
                                                  var_prediction_);
          update_heuristic();
          if (heuristic_pq_.empty()) {
            ROS_ERROR_STREAM("Error! Heuristice priority queue empty!");
            return false;
          }
        }
        std::pair<double, int> interest_point_pair = heuristic_pq_.top();
        res.latitude = location_(interest_point_pair.second, 0) / gp_scale_;
        res.longitude = location_(interest_point_pair.second, 1) / gp_scale_;
        heuristic_pq_.pop();
        return true;
      }
      case DISTANCE_UCB: {
        if (heuristic_pq_v_.empty()) {
          gp_node_.GaussianProcessMixture_predict(location_, mean_prediction_,
                                                  var_prediction_);
          update_heuristic();
          if (heuristic_pq_v_.empty()) {
            ROS_ERROR_STREAM("Error! Heuristice priority queue empty!");
            return false;
          }
        }
        if (!req.robot_id.compare(Jackal_id_) == 0) {
          if (!heuristic_pq_v_[0].empty()) {
            std::pair<double, int> interest_point_pair =
                heuristic_pq_v_[0].top();
            res.latitude = location_(interest_point_pair.second, 0) / gp_scale_;
            res.longitude =
                location_(interest_point_pair.second, 1) / gp_scale_;
            heuristic_pq_v_[0].pop();
            return true;
          } else {
            return false;
          }
        } else if (!req.robot_id.compare(Pelican_id_) == 0) {
          if (!heuristic_pq_v_[1].empty()) {
            std::pair<double, int> interest_point_pair =
                heuristic_pq_v_[1].top();
            res.latitude = location_(interest_point_pair.second, 0) / gp_scale_;
            res.longitude =
                location_(interest_point_pair.second, 1) / gp_scale_;
            heuristic_pq_v_[1].pop();
            return true;
          } else {
            return false;
          }
        } else {
          return false;
        }
      }
      default: { return false; }
    }
    return true;
  }

  void collect_sample_callback(const sampling_msgs::measurement &msg) {
    if (msg.valid) {
      ROS_INFO_STREAM("Master received temperature : " << msg.measurement);
      sample_size_++;
      if (sample_size_ % model_update_rate_ == 0) {
        update_flag_ = true;
      }
      Eigen::MatrixXd new_location, new_feature;
      utils::MsgToMatrix(msg, new_location, new_feature);
      new_location(0, 0) = new_location(0, 0) * gp_scale_;
      new_location(0, 1) = new_location(0, 1) * gp_scale_;
      sample_count_[new_location]++;
      gp_node_.add_training_data(new_location, new_feature);
    } else {
      ROS_INFO_STREAM(
          "Master computer received invalid sample from : " << msg.robot_id);
    }
  }

  void update_heuristic() {
    switch (heuristic_mode_) {
      case VARIANCE: {
        heuristic_pq_ =
            std::priority_queue<std::pair<double, int>,
                                std::vector<std::pair<double, int>>,
                                std::less<std::pair<double, int>>>();
        for (int i = 0; i < location_.rows(); ++i) {
          heuristic_pq_.push(std::make_pair(var_prediction_(i), i));
        }
        break;
      }
      case UCB: {
        heuristic_pq_ =
            std::priority_queue<std::pair<double, int>,
                                std::vector<std::pair<double, int>>,
                                std::less<std::pair<double, int>>>();
        for (int i = 0; i < location_.rows(); ++i) {
          heuristic_pq_.push(std::make_pair(
              mean_prediction_(i) +
                  variance_coeff_ / std::sqrt(sample_count_[location_.row(i)]) *
                      var_prediction_(i),
              i));
        }
        break;
      }
      case DISTANCE_UCB: {
        heuristic_pq_v_.clear();
        Eigen::MatrixXd robot_locations = Eigen::MatrixXd::Zero(2, 2);
        sampling_msgs::RequestLocation srv;
        srv.request.robot_id = Jackal_id_;
        if (!Jackal_GPS_client_.call(srv)) {
          ROS_INFO_STREAM("Can not get Jackal GPS location!");
          break;
        }
        robot_locations(0, 0) = srv.response.latitude;
        robot_locations(0, 1) = srv.response.longitude;
        srv.request.robot_id = Pelican_id_;
        if (!Pelican_GPS_client_.call(srv)) {
          ROS_INFO_STREAM("Can not get Pelican GPS location!");
          break;
        }
        robot_locations(1, 0) = srv.response.latitude;
        robot_locations(1, 1) = srv.response.longitude;

        std::vector<std::vector<int>> labels;
        Eigen::MatrixXd distance;
        voronoi_cell_.UpdateVoronoiMap(robot_locations, distance_scale_factor_,
                                       labels, distance);
        for (size_t i = 0; i < labels.size(); ++i) {
          for (size_t j = 0; j < labels[i].size(); ++j) {
            double q = 0.0;
            const Eigen::MatrixXd point_location = location_.row(labels[i][j]);
            for (const auto &index : labels[i]) {
              double confidence_bound =
                  mean_prediction_(index) +
                  variance_coeff_ /
                      std::sqrt(sample_count_[location_.row(index)]) *
                      var_prediction_(index);
              q = q + L2Distance(point_location, location_.row(index)) *
                          confidence_bound;
            }
            heuristic_pq_v_[i].push(std::make_pair(q, labels[i][j]));
          }
        }
        break;
      }
      default:
        break;
    }
  }

  inline double L2Distance(const Eigen::MatrixXd &location0,
                           const Eigen::MatrixXd &location1) {
    double dx = location0(0, 0) - location1(0, 0);
    double dy = location0(0, 1) - location1(0, 1);
    return dx * dx + dy * dy;
  }

  bool ParseFromRosParam() {
    /// learning data
    XmlRpc::XmlRpcValue data_list;
    if (!rh_.getParam("data_path", data_list)) {
      ROS_ERROR_STREAM("Missing necessary data");
      return false;
    } else {
      if (data_list.size() == 0) {
        ROS_ERROR_STREAM("Empty data path parameters!");
        return false;
      }
      XmlRpc::XmlRpcValue data_path = data_list[0];
      std::string location_data;
      if (!utils::GetParamData(data_path, "location_data", location_)) {
        return false;
      } else {
        for (size_t i = 0; i < location_.rows(); ++i) {
          sample_count_[location_.row(i)] = 1.0;
        }
      }

      if (!utils::GetParamData(data_path, "ground_truth_temperature_data",
                               ground_truth_temperature_)) {
        return false;
      }

      if (!utils::GetParamData(data_path, "initial_location_data",
                               init_sample_location_)) {
        return false;
      }

      if (!utils::GetParamData(data_path, "initial_temperature_data",
                               init_sample_temperature_)) {
        return false;
      }

      ROS_INFO_STREAM("Successfully loaded data!");
    }

    // learning EM learning parameter
    XmlRpc::XmlRpcValue learning_param_list;
    if (!rh_.getParam("learning_parameters", learning_param_list)) {
      ROS_ERROR_STREAM("Missing EM learning parameters");
      return false;
    } else {
      if (learning_param_list.size() == 0) {
        ROS_ERROR_STREAM("Empty learning parameters!");
        return false;
      }
      XmlRpc::XmlRpcValue learning_param = learning_param_list[0];

      if (!utils::GetParam(learning_param, "convergence_threshold",
                           convergence_threshold_)) {
        return false;
      }

      if (!utils::GetParam(learning_param, "max_iteration", max_iteration_)) {
        return false;
      }

      ROS_INFO_STREAM("Successfully loaded EM learning parameters!");
    }

    // learning GP parameter
    XmlRpc::XmlRpcValue gp_param_list;
    if (!rh_.getParam("gp_parameters", gp_param_list)) {
      ROS_ERROR_STREAM("Missing GP parameters");
      return false;
    } else {
      if (gp_param_list.size() == 0) {
        ROS_ERROR_STREAM("Empty GP parameters!");
        return false;
      }
      XmlRpc::XmlRpcValue gp_param = gp_param_list[0];
      if (!utils::GetParam(gp_param, "ground_truth_num_gaussian",
                           ground_truth_num_gaussian_)) {
        return false;
      }

      if (!utils::GetParam(gp_param, "num_gaussian", num_gaussian_)) {
        return false;
      }

      if (!utils::GetParam(gp_param, "gp_hyperparameter", gp_hyperparameter_)) {
        return false;
      }

      if (!utils::GetParam(gp_param, "gp_scale", gp_scale_)) {
        return false;
      }

      ROS_INFO_STREAM("Successfully loaded GP parameters!");
    }

    // learning sampling parameter
    XmlRpc::XmlRpcValue sampling_param_list;
    if (!rh_.getParam("sampling_parameters", sampling_param_list)) {
      ROS_ERROR_STREAM("Missing sampling parameters");
      return false;
    } else {
      if (sampling_param_list.size() == 0) {
        ROS_ERROR_STREAM("Empty sampling parameters!");
        return false;
      }
      XmlRpc::XmlRpcValue sampling_param = sampling_param_list[0];

      int heuristic_mode_int;
      if (!utils::GetParam(sampling_param, "heuristic_mode",
                           heuristic_mode_int)) {
        return false;
      } else {
        switch (heuristic_mode_int) {
          case 0: {
            heuristic_mode_ = VARIANCE;
            break;
          }
          case 1: {
            heuristic_mode_ = UCB;
            break;
          }
          case 2: {
            heuristic_mode_ = DISTANCE_UCB;
            break;
          }
          default: {
            heuristic_mode_ = VARIANCE;
            break;
          }
        }
      }

      if (!utils::GetParam(sampling_param, "Jackal_id", Jackal_id_)) {
        return false;
      }

      if (!utils::GetParam(sampling_param, "Pelican_id", Pelican_id_)) {
        return false;
      }

      if (!utils::GetParam(sampling_param, "scale_factor",
                           distance_scale_factor_)) {
        return false;
      }

      if (!utils::GetParam(sampling_param, "variable_coeff", variance_coeff_)) {
        return false;
      }
      ROS_INFO_STREAM("Successfully loaded sampling parameters!");
    }

    /// visualization node
    XmlRpc::XmlRpcValue visualization_param_list;
    rh_.getParam("visualization_parameters", visualization_param_list);

    for (int32_t i = 0; i < visualization_param_list.size(); ++i) {
      XmlRpc::XmlRpcValue visualization_param = visualization_param_list[i];
      visualization::MAP_PARAM param;
      if (!visualization::GetParam(visualization_param, param)) {
        return false;
      }
      visualization_params_.push_back(param);
    }

    ROS_INFO_STREAM("Finish loading data!");
    return true;
  }

  bool InitializeVisualization() {
    for (const visualization::MAP_PARAM &param : visualization_params_) {
      std::string frame = param.map_frame;
      if (frame.compare("gt") == 0) {
        visualization_node_[frame] =
            std::unique_ptr<visualization::SamplingVisualization>(
                new visualization::SamplingVisualization(nh_, param,
                                                         location_));
      } else if (frame.compare("mean") == 0) {
        visualization_node_[frame] =
            std::unique_ptr<visualization::SamplingVisualization>(
                new visualization::SamplingVisualization(nh_, param,
                                                         location_));
      } else if (frame.compare("variance") == 0) {
        visualization_node_[frame] =
            std::unique_ptr<visualization::SamplingVisualization>(
                new visualization::SamplingVisualization(nh_, param,
                                                         location_));
      } else if (frame.compare("raw") == 0) {
        visualization_node_[frame] =
            std::unique_ptr<visualization::SamplingVisualization>(
                new visualization::SamplingVisualization(
                    nh_, param, init_sample_location_));
      } else {
        ROS_ERROR("Known visualization frame!");
        return false;
      }
    }
    return true;
  }

  void update_gp_model() {
    gp_node_.expectation_maximization(max_iteration_, convergence_threshold_);
    gp_node_.GaussianProcessMixture_predict(location_, mean_prediction_,
                                            var_prediction_);
  }

  void update_visualization() {
    // visualization_node_.update_map(prediction_mean_visualization_offset_,
    //                                mean_prediction_, lowest_temperature_,
    //                                highest_temperature_, heat_map_pred_);
    // if (mean_prediction_.size() > 0) {
    //   distribution_visualization_pub_.publish(heat_map_pred_);
    // }

    // visualization_node_.update_map(prediction_var_visualization_offset_,
    //                                var_prediction_, heat_map_var_);
    // if (var_prediction_.size() > 0) {
    //   distribution_visualization_pub_.publish(heat_map_var_);
    // }

    // if (gt_mean_.size() > 0) {
    //   distribution_visualization_pub_.publish(heat_map_truth_);
    //   distribution_visualization_pub_.publish(heat_map_raw_);
    // }
  }

  // main loop
  void run() {
    update_visualization();
    if (update_flag_) {
      update_flag_ = false;
      update_gp_model();
      update_heuristic();
    }
  }

 private:
  // ROS
  ros::NodeHandle nh_, rh_;
  ros::Publisher distribution_visualization_pub_;
  ros::Subscriber sample_sub_;

  // interest point assignment

  // gp parameter
  int gp_num_gaussian_;
  std::vector<double> gp_hyperparam_;
  double gp_scale_;

  // EM parameter
  double convergence_threshold_;
  int max_iteration_;
  int model_update_rate_;

  // data
  Eigen::MatrixXd location_;
  Eigen::MatrixXd ground_truth_temperature_;
  Eigen::MatrixXd init_sample_location_;
  Eigen::MatrixXd init_sample_temperature_;
  Eigen::VectorXd raw_data_;

  // prediction
  Eigen::VectorXd mean_prediction_;
  Eigen::VectorXd var_prediction_;
  Eigen::VectorXd gt_mean_;
  Eigen::VectorXd gt_var_;

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

};  // namespace sampling
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::CentralizedSamplingNode node(nh, rh);
  while (ros::ok()) {
    node.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
