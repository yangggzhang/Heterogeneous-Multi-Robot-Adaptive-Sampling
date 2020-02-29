#include <ros/package.h>
#include <ros/ros.h>
#include <stdlib.h> /* srand, rand */
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cstdlib>  // std::rand, std::srand
#include <ctime>    // std::time
#include <fstream>
#include <iostream>
#include <sstream>  // std::stringstream
#include <string>

#include "sampling_core/gmm.h"
#include "sampling_core/gpmm.h"
#include "sampling_core/sampling_visualization.h"
#include "sampling_core/utils.h"

float RandomFloat(float a, float b) {
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

std::vector<double> KGPParam{0.5, 0.5, 0.1};

const int KNumberTests = 30;

int random_integer(const int &range) { return std::rand() % range; }

void random_sample_from_dataset(const Eigen::MatrixXd &data_locations,
                                const Eigen::VectorXd &data_measurements,
                                const int &batch_size,
                                Eigen::MatrixXd &batch_locations,
                                Eigen::VectorXd &batch_measurements) {
  assert(batch_size < data_measurements.size());
  const int dataset_size = data_measurements.size();
  std::vector<int> batch_index(batch_size, 0);
  std::srand(unsigned(std::time(0)));
  batch_locations.resize(batch_size, data_locations.cols());
  batch_measurements.resize(batch_size);
  for (int i = 0; i < batch_size; ++i) {
    int random_index = random_integer(dataset_size);
    batch_locations.row(i) = data_locations.row(random_index);
    batch_measurements(i) = data_measurements(random_index);
  }
}

void result_analysis(const std::vector<double> &rmss, double &mean,
                     double &stdev) {
  double sample_size = double(rmss.size());
  double sum = 0.0;
  for (const double &rms : rmss) sum += rms;
  mean = sum / sample_size;
  double dev = 0.0;
  for (const double &rms : rmss) dev += (rms - mean) * (rms - mean);
  stdev = dev / (sample_size - 1.0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gmm_test_node");
  int max_iteration;
  double eps;
  ros::NodeHandle nh("~");
  ros::Publisher distribution_visualization_pub;
  distribution_visualization_pub =
      nh.advertise<visualization_msgs::MarkerArray>("sampling_visualization",
                                                    1);
  // if (!nh.getParam("num_gau", num_gau)) {
  //   num_gau = 3;
  // }
  if (!nh.getParam("max_iteration", max_iteration)) {
    max_iteration = 100;
  }
  if (!nh.getParam("eps", eps)) {
    eps = 0.1;
  }

  Eigen::MatrixXd gt_locations;
  Eigen::VectorXd gt_measures;

  // learning data
  XmlRpc::XmlRpcValue data_list;
  if (!nh.getParam("data_path", data_list)) {
    ROS_ERROR_STREAM("Missing necessary data");
    return -1;
  } else {
    if (data_list.size() == 0) {
      ROS_ERROR_STREAM("Empty data path parameters!");
      return -1;
    }
    XmlRpc::XmlRpcValue data_path = data_list[0];
    if (!sampling::utils::GetParamData(data_path, "gt_locations",
                                       gt_locations)) {
      return -1;
    }
    if (!sampling::utils::GetParamDataVec(data_path, "gt_measurements",
                                          gt_measures)) {
      return -1;
    }
    ROS_INFO_STREAM("Successfully loaded data!");
  }

  std::vector<int> num_gps;
  if (!nh.getParam("num_gps", num_gps)) {
    ROS_ERROR_STREAM("Faild to load number of gps!");
    return -1;
  }

  std::vector<int> num_train_samples;
  if (!nh.getParam("num_train_samples", num_train_samples)) {
    ROS_ERROR_STREAM("Faild to load number of gps!");
    return -1;
  }

  std::ostringstream outss;
  for (const int &num_gp : num_gps) {
    outss << "gp number : " << num_gp << std::endl;
    std::vector<std::vector<double>> gp_params(num_gp, KGPParam);
    sampling::gpmm::GaussianProcessMixtureModel model(num_gp, gp_params,
                                                      max_iteration, eps);
    for (const int &num_train_sample : num_train_samples) {
      outss << "data size : " << num_train_sample << std::endl;
      std::vector<double> batch_results;
      for (int iter = 0; iter < KNumberTests; ++iter) {
        Eigen::MatrixXd training_locations;
        Eigen::VectorXd training_measurements;
        random_sample_from_dataset(gt_locations, gt_measures, num_train_sample,
                                   training_locations, training_measurements);
        model.Train(training_measurements, training_locations);
        Eigen::VectorXd pred_mean, pred_var;
        model.Predict(gt_locations, pred_mean, pred_var);
        double rms = sampling::utils::CalculateRMS(gt_measures, pred_mean);
        batch_results.push_back(rms);
      }
      double mean, stdev;
      result_analysis(batch_results, mean, stdev);
      outss << mean << " " << stdev << std::endl;
    }
  }

  std::ofstream result_file;
  std::string file_dir = ros::package::getPath("sampling_results") +
                         "/gp_test/gp_number_comparison.txt";
  result_file.open(file_dir);
  if (!result_file.is_open()) {
    std::cout << "error opening result.txt" << std::endl;
    return -1;
  } else {
    result_file << outss.str();
    result_file.close();
    std::cout << "Finish test!" << std::endl;
  }

  // std::vector<std::vector<double>> gp_params;
  // XmlRpc::XmlRpcValue gp_param_list;
  // nh.getParam("gp_params", gp_param_list);
  // assert(gp_param_list.size() == num_gau);
  // for (int32_t i = 0; i < gp_param_list.size(); ++i) {
  //   XmlRpc::XmlRpcValue gp_param = gp_param_list[i];
  //   std::vector<double> param;
  //   if (!sampling::utils::GetParam(gp_param, "param", param)) {
  //     ROS_ERROR_STREAM("Failed to load gp param " << i);
  //     return -1;
  //   }
  //   gp_params.push_back(param);
  // }
  // sampling::gpmm::GaussianProcessMixtureModel model(num_gau, gp_params,
  //                                                   max_iteration, eps);
  // ROS_INFO_STREAM("INIT SAMPLE SIZE : " << init_sample_utilities.rows());
  // ROS_INFO_STREAM("INIT POSITION SIZE : " << init_sample_locations.rows() <<
  // " "
  //                                         << init_sample_locations.cols());
  // model.Train(init_sample_utilities, init_sample_locations);
  // Eigen::VectorXd pred_mean, pred_var;
  // model.Predict(test_locations, pred_mean, pred_var);

  // std::vector<sampling::visualization::MAP_PARAM> visualization_params;
  // XmlRpc::XmlRpcValue visualization_param_list;
  // nh.getParam("visualization_parameters", visualization_param_list);
  // for (int32_t i = 0; i < visualization_param_list.size(); ++i) {
  //   XmlRpc::XmlRpcValue visualization_param = visualization_param_list[i];
  //   sampling::visualization::MAP_PARAM param;
  //   if (!sampling::utils::LoadMapParam(visualization_param, param)) {
  //     ROS_ERROR_STREAM("ERROR LOADING VISUALIZATION PARAM!");
  //     return -1;
  //   }
  //   visualization_params.push_back(param);
  // }

  // sampling::visualization::MAP_PARAM robot_param;

  // std::unique_ptr<sampling::visualization::SamplingVisualization>
  // gmm_viz_node =
  //     std::unique_ptr<sampling::visualization::SamplingVisualization>(
  //         new sampling::visualization::SamplingVisualization(
  //             visualization_params, robot_param, 0, test_locations));
  // std::vector<Eigen::VectorXd> viz_values{pred_mean, pred_var};
  // visualization_msgs::MarkerArray marker_array =
  //     gmm_viz_node->UpdateMap(viz_values);
  // ros::Rate r(60);  // 10 hz
  // while (ros::ok()) {
  //   distribution_visualization_pub.publish(marker_array);
  //   ros::spinOnce();
  //   r.sleep();
  // }
  // ros::shutdown();

  return 0;
}
