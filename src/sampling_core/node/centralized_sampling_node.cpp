#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <algorithm>  // std::min_element, std::max_element
#include <string>
#include "sampling_core/gmm_utils.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"

namespace sampling {
class CentralizedSamplingNode {
 public:
  CentralizedSamplingNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    if (!load_parameter()) {
      ROS_ERROR_STREAM("Missing required ros parameter");
    }
    distribution_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "visualization_marker", 10000);
    update_flag_ = false;
    sample_size_ = 0;
    /// initialize visualization
    visualization_node_ = visualization::sampling_visualization(
        ground_truth_location_, visualization_scale_x_, visualization_scale_y_,
        visualization_scale_z_, map_resolution_);
    initialize_visualization();
    gp_node_ = gmm::Gaussian_Mixture_Model(num_gaussian_, gp_hyperparameter_);
  }

  void initialize_visualization() {
    visualization_node_.initialize_map(
        visualization_frame_id_, visualization_namespace_,
        ground_truth_visualization_id_, heat_map_truth_);
    visualization_node_.initialize_map(
        visualization_frame_id_, visualization_namespace_,
        prediction_mean_visualization_id_, heat_map_pred_);
    visualization_node_.initialize_map(
        visualization_frame_id_, visualization_namespace_,
        ground_truth_visualization_id_, heat_map_var_);
    visualization_node_.update_map(ground_truth_visualization_offset_,
                                   ground_truth_temperature_.col(0),
                                   heat_map_truth_);
  }

  void fit_ground_truth_data() {
    gp_node_.add_training_data(ground_truth_location_,
                               ground_truth_temperature_);
    gp_node_.expectation_maximization(max_iteration_, convergence_threshold_);

    gp_node_.GaussianProcessMixture_predict(ground_truth_location_,
                                            mean_prediction_, var_prediction_);
    visualization_node_.update_map(prediction_mean_visualization_offset_,
                                   mean_prediction_, heat_map_pred_);
    visualization_node_.update_map(prediction_var_visualization_offset_,
                                   var_prediction_, heat_map_var_);
  }

  void collect_sample_callback(const sampling_msgs::measurement &msg) {
    if (msg.valid) {
      sample_size_++;
      if (sample_size_ % update_flag_ == 0) {
        update_flag_ = true;
      }
      Eigen::MatrixXd new_location, new_feature;
      utils::MsgToMatrix(msg, new_location, new_feature);
      gp_node_.add_training_data(new_location, new_feature);
      ROS_INFO_STREAM("Master computer successfully collected data!");
    } else {
      ROS_INFO_STREAM(
          "Master computer received invalid sample from : " << msg.robot_id);
    }
  }

  void visualize_distribution() {
    if (mean_prediction_.size() == 0 || var_prediction_.size() == 0) {
      return;
    }

    distribution_visualization_pub_.publish(heat_map_pred_);
    distribution_visualization_pub_.publish(heat_map_var_);
    distribution_visualization_pub_.publish(heat_map_truth_);
  }

  bool load_parameter() {
    bool succeess = true;
    std::string ground_truth_location_path, ground_truth_temperature_path;

    if (!rh_.getParam("ground_truth_location_path",
                      ground_truth_location_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth location data!");
      succeess = false;
    }

    if (!rh_.getParam("ground_truth_temperature_path",
                      ground_truth_temperature_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth temperature data!");
      succeess = false;
    }

    if (!utils::load_ground_truth_data(
            ground_truth_location_path, ground_truth_temperature_path,
            ground_truth_location_, ground_truth_temperature_)) {
      ROS_INFO_STREAM("Error! Can not load ground truth data!");
      succeess = false;
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
          "Error! Missing ground truth data number of gaussian process!");
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
          "Error! Missing prediction mean value visualization map offset in x "
          "direction!");
      succeess = false;
    }

    if (!rh_.getParam("prediction_var_visualization_id",
                      prediction_var_visualization_id_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction variance value visualization map id!");
      succeess = false;
    }

    if (!rh_.getParam("prediction_var_visualization_offset",
                      prediction_var_visualization_offset_)) {
      ROS_INFO_STREAM(
          "Error! Missing prediction variance value visualization map offset "
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

    double min_latitude =
        *std::min_element(latitude_range.begin(), latitude_range.end());
    double max_latitude =
        *std::max_element(latitude_range.begin(), latitude_range.end());
    double min_longitude =
        *std::min_element(longitude_range.begin(), longitude_range.end());
    double max_longitude =
        *std::max_element(longitude_range.begin(), longitude_range.end());
    int num_latitude =
        std::round((max_latitude - min_latitude) / map_resolution_) + 1;
    int num_longitude =
        std::round((max_longitude - min_longitude) / map_resolution_) + 1;
    test_location_ = Eigen::MatrixXd::Zero(num_latitude * num_longitude, 2);
    for (int i = 0; i < num_latitude; ++i) {
      for (int j = 0; j < num_longitude; ++j) {
        int count = i * num_longitude + j;
        test_location_(count, 0) = (double)i * map_resolution_ + min_latitude;
        test_location_(count, 1) = (double)i * map_resolution_ + min_longitude;
      }
    }
    ROS_INFO_STREAM("Finish loading data!");

    /// todo subscribe pelican goal channel
    return succeess;
  }

 private:
  ros::NodeHandle nh_, rh_;
  ros::Publisher distribution_visualization_pub_;
  ros::Subscriber sample_sub_;

  // gp parameter
  int gp_num_gaussian_;
  std::vector<double> gp_hyperparam_;

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

  Eigen::MatrixXd sample_location_;
  Eigen::MatrixXd sample_temperature_;
  Eigen::MatrixXd test_location_;

  Eigen::VectorXd mean_prediction_;
  Eigen::VectorXd var_prediction_;

  // GP parameter
  int num_gaussian_;
  std::vector<double> gp_hyperparameter_;
  gmm::Gaussian_Mixture_Model gp_node_;
  gmm::Model gt_model_;
  gmm::Model model_;

  visualization_msgs::Marker heat_map_pred_;
  visualization_msgs::Marker heat_map_var_;
  visualization_msgs::Marker heat_map_truth_;

  /// visualization
  std::string visualization_frame_id_;
  std::string visualization_namespace_;
  int ground_truth_visualization_id_;
  int ground_truth_visualization_offset_;
  int prediction_mean_visualization_id_;
  int prediction_mean_visualization_offset_;
  int prediction_var_visualization_id_;
  int prediction_var_visualization_offset_;
  double visualization_scale_x_, visualization_scale_y_, visualization_scale_z_,
      map_resolution_;

  visualization::sampling_visualization visualization_node_;
};
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::CentralizedSamplingNode node(nh, rh);
  node.fit_ground_truth_data();
  while (ros::ok()) {
    node.visualize_distribution();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
