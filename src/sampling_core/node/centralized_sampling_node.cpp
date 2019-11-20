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
    // Load parameters
    if (!load_parameter()) {
      ROS_ERROR_STREAM("Missing required ros parameter");
    }

    // Visualization
    distribution_visualization_pub_ =
        nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    interest_point_assignment_ser_ = nh_.advertiseService(
        interest_point_service_channel_,
        &CentralizedSamplingNode::assign_interest_point, this);
    visualization_node_ = visualization::sampling_visualization(
        visualization_scale_x_, visualization_scale_y_, visualization_scale_z_,
        num_lat_, num_lng_);
    ROS_INFO_STREAM("Map range : " << num_lat_ << " " << num_lng_);
    visualization_node_.initialize_map(
        visualization_frame_id_, visualization_namespace_,
        prediction_mean_visualization_id_, heat_map_pred_);
    visualization_node_.initialize_map(
        visualization_frame_id_, visualization_namespace_,
        prediction_var_visualization_id_, heat_map_var_);

    // Sampling
    sample_sub_ =
        nh_.subscribe(temperature_update_channel_, 1,
                      &CentralizedSamplingNode::collect_sample_callback, this);
    voronoi_cell_ = voronoi::Voronoi(test_location_);
    update_flag_ = false;
    sample_size_ = 0;

    // GP
    gp_node_ = gmm::Gaussian_Mixture_Model(num_gaussian_, gp_hyperparameter_);
    gt_gp_node_ = gp_node_;

    // Robot agent
    Jackal_GPS_client_ = nh_.serviceClient<sampling_msgs::RequestLocation>(
        Jackal_request_GPS_channel_);

    Pelican_GPS_client_ = nh_.serviceClient<sampling_msgs::RequestLocation>(
        Pelican_request_GPS_channel_);

    // Initial data
    if (init_sample_temperature_.rows() > 0) {
      gp_node_.add_training_data(init_sample_location_,
                                 init_sample_temperature_);
      update_gp_model();
      update_heuristic();
      ROS_INFO_STREAM("Initialize GP model with initial data points");
    }

    if (ground_truth_location_.rows() > 0 &&
        ground_truth_temperature_.rows() > 0) {
      ROS_INFO_STREAM("Update ground truth model!");
      visualization_node_.initialize_map(
          visualization_frame_id_, visualization_namespace_,
          ground_truth_visualization_id_, heat_map_truth_);
      gt_gp_node_.add_training_data(ground_truth_location_,
                                    ground_truth_temperature_);
      gt_gp_node_.expectation_maximization(max_iteration_,
                                           convergence_threshold_);
      gt_gp_node_.GaussianProcessMixture_predict(test_location_, gt_mean_,
                                                 gt_var_);
      ROS_INFO_STREAM("GP max : " << gt_mean_.maxCoeff()
                                  << " min value : " << gt_mean_.minCoeff());
      visualization_node_.update_map(ground_truth_visualization_offset_,
                                     gt_mean_, lowest_temperature_,
                                     highest_temperature_, heat_map_truth_);

      raw_data_ = init_sample_temperature_.col(0).array();

      visualization_node_.initialize_map(
          visualization_frame_id_, visualization_namespace_,
          raw_data_visualization_id_, heat_map_raw_);
      visualization_node_.update_map(raw_data_visualization_offset_, raw_data_,
                                     lowest_temperature_, highest_temperature_,
                                     heat_map_raw_);
    }
  }

  bool assign_interest_point(sampling_msgs::RequestGoal::Request &req,
                             sampling_msgs::RequestGoal::Response &res) {
    ROS_INFO_STREAM(
        "Master Computer received request from robot : " << req.robot_id);
    // todo \yang add hetegeneous functionality
    switch (heuristic_mode_) {
      case VARIANCE: {
        if (heuristic_pq_.empty()) {
          gp_node_.GaussianProcessMixture_predict(
              test_location_, mean_prediction_, var_prediction_);
          update_heuristic();
          if (heuristic_pq_.empty()) {
            ROS_ERROR_STREAM("Error! Heuristice priority queue empty!");
            return false;
          }
        }
        std::pair<double, int> interest_point_pair = heuristic_pq_.top();
        res.latitude = test_location_(interest_point_pair.second, 0);
        res.longitude = test_location_(interest_point_pair.second, 1);
        heuristic_pq_.pop();
        return true;
      }
      case UCB: {
        if (heuristic_pq_.empty()) {
          gp_node_.GaussianProcessMixture_predict(
              test_location_, mean_prediction_, var_prediction_);
          update_heuristic();
          if (heuristic_pq_.empty()) {
            ROS_ERROR_STREAM("Error! Heuristice priority queue empty!");
            return false;
          }
        }
        std::pair<double, int> interest_point_pair = heuristic_pq_.top();
        res.latitude = test_location_(interest_point_pair.second, 0);
        res.longitude = test_location_(interest_point_pair.second, 1);
        heuristic_pq_.pop();
        return true;
      }
      case DISTANCE_UCB: {
        if (heuristic_pq_v_.empty()) {
          gp_node_.GaussianProcessMixture_predict(
              test_location_, mean_prediction_, var_prediction_);
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
            res.latitude = test_location_(interest_point_pair.second, 0);
            res.longitude = test_location_(interest_point_pair.second, 1);
            heuristic_pq_v_[0].pop();
            return true;
          } else {
            return false;
          }
        } else if (!req.robot_id.compare(Pelican_id_) == 0) {
          if (!heuristic_pq_v_[1].empty()) {
            std::pair<double, int> interest_point_pair =
                heuristic_pq_v_[1].top();
            res.latitude = test_location_(interest_point_pair.second, 0);
            res.longitude = test_location_(interest_point_pair.second, 1);
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
      sample_count_[new_location]++;
      gp_node_.add_training_data(new_location, new_feature);
    } else {
      ROS_INFO_STREAM(
          "Master computer received invalid sample from : " << msg.robot_id);
    }
  }

  void visualize_distribution() {
    if (mean_prediction_.size() == 0 || var_prediction_.size() == 0) {
      return;
    }
  }

  void update_heuristic() {
    switch (heuristic_mode_) {
      case VARIANCE: {
        heuristic_pq_ =
            std::priority_queue<std::pair<double, int>,
                                std::vector<std::pair<double, int>>,
                                std::less<std::pair<double, int>>>();
        for (int i = 0; i < test_location_.rows(); ++i) {
          heuristic_pq_.push(std::make_pair(var_prediction_(i), i));
        }
        break;
      }
      case UCB: {
        heuristic_pq_ =
            std::priority_queue<std::pair<double, int>,
                                std::vector<std::pair<double, int>>,
                                std::less<std::pair<double, int>>>();
        for (int i = 0; i < test_location_.rows(); ++i) {
          heuristic_pq_.push(std::make_pair(
              mean_prediction_(i) +
                  variance_coeff_ /
                      std::sqrt(sample_count_[test_location_.row(i)]) *
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
            const Eigen::MatrixXd point_location =
                test_location_.row(labels[i][j]);
            for (const auto &index : labels[i]) {
              double confidence_bound =
                  mean_prediction_(index) +
                  variance_coeff_ /
                      std::sqrt(sample_count_[test_location_.row(index)]) *
                      var_prediction_(index);
              q = q + L2Distance(point_location, test_location_.row(index)) *
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

  bool load_parameter() {
    bool succeess = true;
    std::string ground_truth_location_path, ground_truth_temperature_path,
        initial_location_path, initial_temperature_path;

    if (!rh_.getParam("ground_truth_location_path",
                      ground_truth_location_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth location data!");
    }

    if (!rh_.getParam("ground_truth_temperature_path",
                      ground_truth_temperature_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth temperature data!");
    }

    if (!ground_truth_location_path.empty() &&
        !ground_truth_temperature_path.empty()) {
      if (!utils::load_data(
              ground_truth_location_path, ground_truth_temperature_path,
              ground_truth_location_, ground_truth_temperature_)) {
        ROS_INFO_STREAM("Error! Can not load ground truth data!");
      }
    }

    if (!rh_.getParam("initial_location_path", initial_location_path)) {
      ROS_INFO_STREAM("Error! Missing initial location data!");
    }

    if (!rh_.getParam("initial_temperature_path", initial_temperature_path)) {
      ROS_INFO_STREAM("Error! Missing initial temperature data!");
    }

    if (!utils::load_data(initial_location_path, initial_temperature_path,
                          init_sample_location_, init_sample_temperature_)) {
      ROS_INFO_STREAM("Error! Can not initialize sampling data!");
    }

    if (!rh_.getParam("convergence_threshold", convergence_threshold_)) {
      ROS_INFO_STREAM("Error! Missing EM convergence threshold!");
      succeess = false;
    }

    if (!rh_.getParam("max_iteration", max_iteration_)) {
      ROS_INFO_STREAM("Error! Missing EM maximum iteration!");
      succeess = false;
    }

    if (!rh_.getParam("ground_truth_num_gaussian",
                      ground_truth_num_gaussian_)) {
      ROS_INFO_STREAM(
          "Error! Missing ground truth data number of gaussian "
          "process!");
      succeess = false;
    }

    if (!rh_.getParam("temperature_update_channel",
                      temperature_update_channel_)) {
      ROS_INFO_STREAM("Error! Missing temperature sample update channel!");
      succeess = false;
    }

    if (!rh_.getParam("model_update_rate", model_update_rate_)) {
      ROS_INFO_STREAM("Error! Missing model update rate!");
      succeess = false;
    }

    if (!rh_.getParam("visualization_frame_id", visualization_frame_id_)) {
      ROS_INFO_STREAM("Error! Missing visualization frame id!");
      succeess = false;
    }

    if (!rh_.getParam("visualization_namespace", visualization_namespace_)) {
      ROS_INFO_STREAM("Error! Missing visualization namespace!");
      succeess = false;
    }

    if (!rh_.getParam("map_resolution", map_resolution_)) {
      ROS_INFO_STREAM("Error! Missing visualization map resolution!");
      succeess = false;
    }

    if (!rh_.getParam("ground_truth_visualization_id",
                      ground_truth_visualization_id_)) {
      ROS_INFO_STREAM("Error! Missing ground truth visualization map id!");
      succeess = false;
    }

    if (!rh_.getParam("ground_truth_visualization_offset",
                      ground_truth_visualization_offset_)) {
      ROS_INFO_STREAM("Error! Missing ground truth visualization map offset! ");
      succeess = false;
    }

    if (!rh_.getParam("prediction_mean_visualization_id",
                      prediction_mean_visualization_id_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction mean value visualization map id!");
      succeess = false;
    }

    if (!rh_.getParam("prediction_mean_visualization_offset",
                      prediction_mean_visualization_offset_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction mean value visualization map "
          "offset "
          "in "
          "x "
          "direction!");
      succeess = false;
    }

    if (!rh_.getParam("prediction_var_visualization_id",
                      prediction_var_visualization_id_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction variance value visualization map "
          "id!");
      succeess = false;
    }

    if (!rh_.getParam("prediction_var_visualization_offset",
                      prediction_var_visualization_offset_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction variance value visualization map "
          "offset "
          "in x direction!");
      succeess = false;
    }

    if (!rh_.getParam("raw_data_visualization_id",
                      raw_data_visualization_id_)) {
      ROS_INFO_STREAM(
          "Error! Missing raw data visualization map "
          "id!");
      succeess = false;
    }

    if (!rh_.getParam("raw_data_visualization_offset",
                      raw_data_visualization_offset_)) {
      ROS_INFO_STREAM(
          "Error! Missing raw data value visualization map "
          "offset "
          "in x direction!");
      succeess = false;
    }

    if (!rh_.getParam("visualization_scale_x", visualization_scale_x_)) {
      ROS_INFO_STREAM("Error! Missing visualization scale in x direction!");
      succeess = false;
    }

    if (!rh_.getParam("visualization_scale_y", visualization_scale_y_)) {
      ROS_INFO_STREAM("Error! Missing visualization scale in y direction!");
      succeess = false;
    }

    if (!rh_.getParam("visualization_scale_z", visualization_scale_z_)) {
      ROS_INFO_STREAM("Error! Missing visualization scale in z direction!");
      succeess = false;
    }

    if (!rh_.getParam("num_gaussian", num_gaussian_)) {
      ROS_INFO_STREAM("Error! Missing number of gaussian process!");
      succeess = false;
    }

    if (!rh_.getParam("gp_hyperparameter", gp_hyperparameter_)) {
      ROS_INFO_STREAM("Error! Missing gaussian process hyperparameter!");
      succeess = false;
    }

    std::vector<double> latitude_range, longitude_range;

    if (!rh_.getParam("latitude_range", latitude_range)) {
      ROS_INFO_STREAM("Error! Missing test field latitude_range!");
      succeess = false;
    }

    if (!rh_.getParam("longitude_range", longitude_range)) {
      ROS_INFO_STREAM("Error! Missing test field longitude range!");
      succeess = false;
    }

    if (!rh_.getParam("interest_point_request_service_channel",
                      interest_point_service_channel_)) {
      ROS_INFO_STREAM("Error! Missing interest point request service channel!");
      succeess = false;
    }

    int heuristic_mode_int;

    if (!rh_.getParam("heuristic_mode", heuristic_mode_int)) {
      ROS_INFO_STREAM("Error! Missing interest assignment heuristic mode!");
      succeess = false;
    }
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

    if (!rh_.getParam("Jackal_request_GPS_channel",
                      Jackal_request_GPS_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal request GPS channel!");
      succeess = false;
    }

    if (!rh_.getParam("Pelican_request_GPS_channel",
                      Pelican_request_GPS_channel_)) {
      ROS_INFO_STREAM("Error! Missing Pelican request GPS channel!");
      succeess = false;
    }

    if (!rh_.getParam("Jackal_id", Jackal_id_)) {
      ROS_INFO_STREAM("Error! Missing Jackal id!");
      succeess = false;
    }

    if (!rh_.getParam("Pelican_id", Pelican_id_)) {
      ROS_INFO_STREAM("Error! Missing Pelican id!");
      succeess = false;
    }

    std::vector<double> scale_factor_v;
    if (!rh_.getParam("scale_factor", scale_factor_v)) {
      ROS_INFO_STREAM("Error! Missing scale factor!");
      succeess = false;
    }
    distance_scale_factor_.resize(scale_factor_v.size());
    for (size_t i = 0; i < scale_factor_v.size(); ++i) {
      distance_scale_factor_(i) = scale_factor_v[i];
    }

    if (!rh_.getParam("variable_coeff", variance_coeff_)) {
      ROS_INFO_STREAM("Error! Missing variance coefficient for UCB!");
      succeess = false;
    }

    if (!rh_.getParam("lowest_temperature", lowest_temperature_)) {
      ROS_INFO_STREAM("Error! Missing lowerest temperature for visualization!");
      succeess = false;
    }

    if (!rh_.getParam("highest_temperature", highest_temperature_)) {
      ROS_INFO_STREAM("Error! Missing highest temperature for visualization!");
      succeess = false;
    }

    double min_latitude =
        *std::min_element(latitude_range.begin(), latitude_range.end());
    double max_latitude =
        *std::max_element(latitude_range.begin(), latitude_range.end());
    double min_longitude =
        *std::min_element(longitude_range.begin(), longitude_range.end());
    double max_longitude =
        *std::max_element(longitude_range.begin(), longitude_range.end());

    num_lat_ = std::round((max_latitude - min_latitude) / map_resolution_) + 1;
    num_lng_ =
        std::round((max_longitude - min_longitude) / map_resolution_) + 1;

    test_location_ = Eigen::MatrixXd::Zero(num_lat_ * num_lng_, 2);
    for (int i = 0; i < num_lat_; ++i) {
      for (int j = 0; j < num_lng_; ++j) {
        int count = i * num_lng_ + j;
        test_location_(count, 0) = (double)i * map_resolution_ + min_latitude;
        test_location_(count, 1) = (double)j * map_resolution_ + min_longitude;
        sample_count_[test_location_.row(count)] = 1.0;
      }
    }

    ROS_INFO_STREAM("Finish loading data!");

    /// todo subscribe pelican goal channel
    return succeess;
  }

  void update_gp_model() {
    gp_node_.expectation_maximization(max_iteration_, convergence_threshold_);
    gp_node_.GaussianProcessMixture_predict(test_location_, mean_prediction_,
                                            var_prediction_);
  }

  void update_visualization() {
    visualization_node_.update_map(prediction_mean_visualization_offset_,
                                   mean_prediction_, lowest_temperature_,
                                   highest_temperature_, heat_map_pred_);
    if (mean_prediction_.size() > 0) {
      distribution_visualization_pub_.publish(heat_map_pred_);
    }

    visualization_node_.update_map(prediction_var_visualization_offset_,
                                   var_prediction_, heat_map_var_);
    if (var_prediction_.size() > 0) {
      distribution_visualization_pub_.publish(heat_map_var_);
    }

    if (gt_mean_.size() > 0) {
      distribution_visualization_pub_.publish(heat_map_truth_);
      distribution_visualization_pub_.publish(heat_map_raw_);
    }
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
  ros::NodeHandle nh_, rh_;
  ros::Publisher distribution_visualization_pub_;
  ros::Subscriber sample_sub_;

  // interest point assignment
  std::string interest_point_service_channel_;
  ros::ServiceServer interest_point_assignment_ser_;
  HeuristicMode heuristic_mode_;

  // gp parameter
  int gp_num_gaussian_;
  std::vector<double> gp_hyperparam_;
  pq heuristic_pq_;
  std::vector<pq> heuristic_pq_v_;

  int num_lat_, num_lng_;

  // GroundTruthData ground_truth_data_;
  double convergence_threshold_;
  int max_iteration_;
  int ground_truth_num_gaussian_;

  int model_update_rate_;
  bool update_flag_;
  int sample_size_;

  std::string temperature_update_channel_;

  Eigen::MatrixXd ground_truth_location_;
  Eigen::MatrixXd ground_truth_temperature_;

  Eigen::MatrixXd init_sample_location_, init_sample_temperature_;
  Eigen::MatrixXd test_location_;

  Eigen::VectorXd gt_mean_;
  Eigen::VectorXd gt_var_;

  Eigen::VectorXd mean_prediction_;
  Eigen::VectorXd var_prediction_;

  Eigen::VectorXd raw_data_;

  // GP parameter
  int num_gaussian_;
  std::vector<double> gp_hyperparameter_;
  gmm::Gaussian_Mixture_Model gp_node_;
  gmm::Gaussian_Mixture_Model gt_gp_node_;
  gmm::Model gt_model_;
  gmm::Model model_;

  visualization_msgs::Marker heat_map_pred_;
  visualization_msgs::Marker heat_map_var_;
  visualization_msgs::Marker heat_map_truth_;
  visualization_msgs::Marker heat_map_raw_;

  /// visualization
  std::string visualization_frame_id_;
  std::string visualization_namespace_;
  int ground_truth_visualization_id_;
  int ground_truth_visualization_offset_;
  int prediction_mean_visualization_id_;
  int prediction_mean_visualization_offset_;
  int prediction_var_visualization_id_;
  int prediction_var_visualization_offset_;
  int raw_data_visualization_id_;
  int raw_data_visualization_offset_;

  double visualization_scale_x_, visualization_scale_y_, visualization_scale_z_,
      map_resolution_;

  visualization::sampling_visualization visualization_node_;

  voronoi::Voronoi voronoi_cell_;
  std::string Jackal_request_GPS_channel_, Pelican_request_GPS_channel_,
      Jackal_id_, Pelican_id_;
  ros::ServiceClient Jackal_GPS_client_;
  ros::ServiceClient Pelican_GPS_client_;
  Eigen::VectorXd distance_scale_factor_;
  double variance_coeff_;

  std::unordered_map<Eigen::MatrixXd, double, GPSHashFunction> sample_count_;

  double lowest_temperature_;
  double highest_temperature_;

};  // namespace sampling
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(100);
  sampling::CentralizedSamplingNode node(nh, rh);
  // node.fit_ground_truth_data();
  while (ros::ok()) {
    node.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
