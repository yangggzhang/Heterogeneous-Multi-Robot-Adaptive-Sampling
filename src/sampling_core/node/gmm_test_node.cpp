#include <ros/package.h>
#include <ros/ros.h>
#include <stdlib.h> /* srand, rand */
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <iostream>
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

bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode,
                  sampling::visualization::MAP_PARAM &param) {
  if (!sampling::utils::GetParam(YamlNode, "map_frame", param.map_frame)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "map_id", param.map_id)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "x_scale", param.x_scale)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "y_scale", param.y_scale)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "x_offset", param.x_offset)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "y_offset", param.y_offset)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "lower_bound", param.lower_bound)) {
    return false;
  }
  if (!sampling::utils::GetParam(YamlNode, "upper_bound", param.upper_bound)) {
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gmm_test_node");
  int num_gau, max_iteration;
  double eps;
  ros::NodeHandle nh("~");
  ros::Publisher distribution_visualization_pub;
  distribution_visualization_pub =
      nh.advertise<visualization_msgs::MarkerArray>("sampling_visualization",
                                                    1);
  if (!nh.getParam("num_gau", num_gau)) {
    num_gau = 3;
  }
  if (!nh.getParam("max_iteration", max_iteration)) {
    max_iteration = 100;
  }
  if (!nh.getParam("eps", eps)) {
    eps = 0.1;
  }

  Eigen::VectorXd init_sample_utilities;
  Eigen::MatrixXd init_sample_locations, test_locations;
  std::string init_sample_path, init_location_path, test_location_path;
  if (!nh.getParam("init_sample_path", init_sample_path)) {
    ROS_ERROR_STREAM("Missing initial sample path!");
    return -1;
  } else {
    std::string file_path =
        ros::package::getPath("sampling_data") + "/data/" + init_sample_path;

    if (!sampling::utils::LoadDataVec(file_path, init_sample_utilities)) {
      ROS_ERROR_STREAM("Failed to load initial sample!");
      return -1;
    }
  }

  if (!nh.getParam("init_location_path", init_location_path)) {
    ROS_ERROR_STREAM("Missing initial sample location path!");
    return -1;
  } else {
    std::string file_path =
        ros::package::getPath("sampling_data") + "/data/" + init_location_path;

    if (!sampling::utils::LoadData(file_path, init_sample_locations)) {
      ROS_ERROR_STREAM("Failed to load initial sample locations!");
      return -1;
    }
  }

  if (!nh.getParam("test_location_path", test_location_path)) {
    ROS_ERROR_STREAM("Missing test sample location path!");
    return -1;
  } else {
    std::string file_path =
        ros::package::getPath("sampling_data") + "/data/" + test_location_path;

    if (!sampling::utils::LoadData(file_path, test_locations)) {
      ROS_ERROR_STREAM("Failed to load test locations!");
      return -1;
    }
  }

  std::vector<std::vector<double>> gp_params;
  XmlRpc::XmlRpcValue gp_param_list;
  nh.getParam("gp_params", gp_param_list);
  assert(gp_param_list.size() == num_gau);
  for (int32_t i = 0; i < gp_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue gp_param = gp_param_list[i];
    std::vector<double> param;
    if (!sampling::utils::GetParam(gp_param, "param", param)) {
      ROS_ERROR_STREAM("Failed to load gp param " << i);
      return -1;
    }
    gp_params.push_back(param);
  }
  sampling::gpmm::GaussianProcessMixtureModel model(num_gau, gp_params,
                                                    max_iteration, eps);
  ROS_INFO_STREAM("INIT SAMPLE SIZE : " << init_sample_utilities.rows());
  ROS_INFO_STREAM("INIT POSITION SIZE : " << init_sample_locations.rows() << " "
                                          << init_sample_locations.cols());
  model.Train(init_sample_utilities, init_sample_locations);
  Eigen::VectorXd pred_mean, pred_var;
  model.Predict(test_locations, pred_mean, pred_var);
  // for (int i = 0; i < pred_mean.size(); ++i) {
  //   ROS_INFO_STREAM("Sample : " << i << " Mean : " << pred_mean(i)
  //                               << " Var : " << pred_var(i));
  // }

  std::vector<sampling::visualization::MAP_PARAM> visualization_params;
  XmlRpc::XmlRpcValue visualization_param_list;
  nh.getParam("visualization_parameters", visualization_param_list);
  for (int32_t i = 0; i < visualization_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue visualization_param = visualization_param_list[i];
    sampling::visualization::MAP_PARAM param;
    if (!LoadMapParam(visualization_param, param)) {
      ROS_ERROR_STREAM("ERROR LOADING VISUALIZATION PARAM!");
      return -1;
    }
    visualization_params.push_back(param);
  }

  std::unique_ptr<sampling::visualization::SamplingVisualization> mean_viz_node,
      var_viz_node;
  for (const sampling::visualization::MAP_PARAM &viz_param :
       visualization_params) {
    std::string frame_str = viz_param.map_frame;
    if (frame_str.compare("mean") == 0) {
      mean_viz_node =
          std::unique_ptr<sampling::visualization::SamplingVisualization>(
              new sampling::visualization::SamplingVisualization(
                  nh, viz_param, test_locations));
    }
    if (frame_str.compare("variance") == 0) {
      var_viz_node =
          std::unique_ptr<sampling::visualization::SamplingVisualization>(
              new sampling::visualization::SamplingVisualization(
                  nh, viz_param, test_locations));
    }
  }
  mean_viz_node->UpdateMap(pred_mean);
  var_viz_node->UpdateMap(pred_var);
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(mean_viz_node->GetMarker());
  marker_array.markers.push_back(var_viz_node->GetMarker());
  ros::Rate r(60);  // 10 hz
  while (ros::ok()) {
    distribution_visualization_pub.publish(marker_array);
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}