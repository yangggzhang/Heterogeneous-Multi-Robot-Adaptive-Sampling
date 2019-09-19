#include "sampling_core/gmm_utils.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"
#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <string>

namespace sampling {
class CentralizedSamplingNode {
public:
  CentralizedSamplingNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    load_parameter();
    distribution_visualization_pub_ =
        nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    update_flag_;
  }

  void fit_ground_truth_data() {
    gt_model_.numGaussian = ground_truth_num_gaussian_;
    gt_model_.R = Eigen::MatrixXd::Random(ground_truth_temperature_.rows(),
                                          ground_truth_num_gaussian_);
    gt_model_.R = gt_model_.R.array().abs();
    gmm::expectation_maximization(ground_truth_temperature_, max_iteration_,
                                  convergence_threshold_, gt_model_);

    gmm::GaussianProcessMixture_predict(
        ground_truth_location_, ground_truth_temperature_,
        ground_truth_location_, gt_model_, mean_prediction_, var_prediction_);
  }

  void collect_sample_callback(const sampling_msgs::measurement &msg) {
    if (msg.valid) {
      sample_size_++;
      sample_temperature_.conservativeResize(sample_size_, 1);
      sample_location_.conservativeResize(sample_size_, 2);
      sample_temperature_(sample_size_, 0) = msg.measurement;
      sample_location_(sample_size_, 0) = msg.latitude;
      sample_location_(sample_size_, 1) = msg.longitude;
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

    visualization::construct_visualization_map(
        ground_truth_temperature_, mean_prediction_, var_prediction_,
        seed_point_, heat_map_pred_, heat_map_var_, heat_map_truth_);

    distribution_visualization_pub_.publish(seed_point_);
    distribution_visualization_pub_.publish(heat_map_pred_);
    distribution_visualization_pub_.publish(heat_map_var_);
    distribution_visualization_pub_.publish(heat_map_truth_);
  }

  bool load_parameter() {
    std::string ground_truth_location_path, ground_truth_temperature_path;

    if (!rh_.getParam("ground_truth_location_path",
                      ground_truth_location_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth location data!");
      return false;
    }

    if (!rh_.getParam("ground_truth_temperature_path",
                      ground_truth_temperature_path)) {
      ROS_INFO_STREAM("Error! Missing ground truth temperature data!");
      return false;
    }

    if (!utils::load_ground_truth_data(
            ground_truth_location_path, ground_truth_temperature_path,
            ground_truth_location_, ground_truth_temperature_)) {
      ROS_INFO_STREAM("Error! Can not load ground truth data!");
      return false;
    }

    if (!rh_.getParam("convergence_threshold", convergence_threshold_)) {
      ROS_INFO_STREAM("Error! Missing EM convergence threshold!");
      return false;
    }

    if (!rh_.getParam("max_iteration", max_iteration_)) {
      ROS_INFO_STREAM("Error! Missing EM maximum iteration!");
      return false;
    }

    if (!rh_.getParam("ground_truth_num_gaussian",
                      ground_truth_num_gaussian_)) {
      ROS_INFO_STREAM(
          "Error! Missing ground truth data number of gaussian process!");
      return false;
    }

    if (!rh_.getParam("temperature_update_channel",
                      temperature_update_channel_)) {
      ROS_INFO_STREAM("Error! Missing temperature sample update channel!");
      return false;
    }

    if (!rh_.getParam("model_update_rate", model_update_rate_)) {
      ROS_INFO_STREAM("Error! Missing model update rate!");
      return false;
    }

    ROS_INFO_STREAM("Finish loading data!");

    /// todo subscribe pelican goal channel
    return true;
  }

private:
  ros::NodeHandle nh_, rh_;
  ros::Publisher distribution_visualization_pub_;
  ros::Subscriber sample_sub_;

  // GroundTruthData ground_truth_data_;
  double convergence_threshold_;
  int max_iteration_;
  int ground_truth_num_gaussian_;

  int model_update_rate_;
  bool update_flag_;

  std::string temperature_update_channel_;

  Eigen::MatrixXd ground_truth_location_;
  Eigen::MatrixXd ground_truth_temperature_;

  Eigen::MatrixXd sample_location_;
  Eigen::MatrixXd sample_temperature_;
  size_t sample_size_;

  Eigen::VectorXd mean_prediction_;
  Eigen::VectorXd var_prediction_;

  gmm::Model gt_model_;
  gmm::Model model_;

  visualization_msgs::Marker seed_point_;
  visualization_msgs::Marker heat_map_pred_;
  visualization_msgs::Marker heat_map_var_;
  visualization_msgs::Marker heat_map_truth_;
};
} // namespace sampling

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
