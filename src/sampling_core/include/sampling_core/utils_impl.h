#include <ros/package.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "utils.h"
namespace sampling {
namespace utils {

bool LoadData(const std::string &path, Eigen::MatrixXd &data) {
  std::ifstream file(path.c_str(), std::ifstream::in);

  if (!file.is_open()) {
    ROS_INFO_STREAM("open Fss File: Error opening file" << path);
    return false;
  }

  std::vector<std::vector<double>> data_vec;

  std::string new_line;

  while (getline(file, new_line)) {
    std::stringstream ss(new_line);
    std::vector<double> new_data;
    for (double k; ss >> k;) {
      new_data.push_back(k);
      if (ss.peek() == ',') ss.ignore();
    }
    data_vec.push_back(new_data);
  }

  if (data_vec.empty()) {
    ROS_ERROR_STREAM("Empty data!");
    return false;
  }

  data.resize(data_vec.size(), data_vec.front().size());
  for (int i = 0; i < data_vec.size(); ++i) {
    for (int j = 0; j < data_vec[i].size(); ++j) {
      data(i, j) = data_vec[i][j];
    }
  }
  return true;
}

bool LoadDataVec(const std::string &path, Eigen::VectorXd &data) {
  Eigen::MatrixXd data_mat;
  if (!LoadData(path, data_mat)) {
    ROS_ERROR_STREAM("Failed to load " << path);
    return false;
  }
  if (data_mat.cols() != 1) {
    ROS_ERROR_STREAM("Loaded data has multiple dimensions!");
    return false;
  }
  data = data_mat.col(0);
  return true;
}

bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode,
                  sampling::visualization::MAP_PARAM &param) {
  if (!GetParam(YamlNode, "map_frame", param.map_frame)) {
    return false;
  }
  if (!GetParam(YamlNode, "map_id", param.map_id)) {
    return false;
  }
  if (!GetParam(YamlNode, "x_scale", param.x_scale)) {
    return false;
  }
  if (!GetParam(YamlNode, "y_scale", param.y_scale)) {
    return false;
  }
  if (!GetParam(YamlNode, "x_offset", param.x_offset)) {
    return false;
  }
  if (!GetParam(YamlNode, "y_offset", param.y_offset)) {
    return false;
  }
  if (!GetParam(YamlNode, "lower_bound", param.lower_bound)) {
    return false;
  }
  if (!GetParam(YamlNode, "upper_bound", param.upper_bound)) {
    return false;
  }
  return true;
}

bool GetParamData(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
                  Eigen::MatrixXd &data) {
  std::string file_dir;
  if (!GetParam(YamlNode, param_name, file_dir)) {
    return false;
  } else {
    file_dir = ros::package::getPath("sampling_data") + "/data/" + file_dir;
    if (!LoadData(file_dir, data)) {
      return false;
    }
  }
  return true;
}

bool GetParamDataVec(XmlRpc::XmlRpcValue &YamlNode,
                     const std::string &param_name, Eigen::VectorXd &data) {
  std::string file_dir;
  if (!GetParam(YamlNode, param_name, file_dir)) {
    return false;
  } else {
    file_dir = ros::package::getPath("sampling_data") + "/data/" + file_dir;
    if (!LoadDataVec(file_dir, data)) {
      return false;
    }
  }
  return true;
}

void MsgToMatrix(const sampling_msgs::measurement &msg,
                 Eigen::MatrixXd &location, Eigen::MatrixXd &feature) {
  location = Eigen::MatrixXd::Zero(1, 2);
  feature = Eigen::MatrixXd::Zero(1, 1);
  location(0, 0) = msg.location_x;
  location(0, 1) = msg.location_y;
  feature(0, 0) = msg.measurement;
}

double CalculateRMS(Eigen::VectorXd &pred, Eigen::VectorXd &gt) {
  Eigen::VectorXd diff = pred - gt;
  double rms = diff.squaredNorm() / (double)diff.size();
  return std::sqrt(rms);
}

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              T &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_INFO_STREAM("Missing parameter : " << param_name);
    return false;
  } else {
    data = static_cast<T>(YamlNode[param_name]);
  }
  return true;
}

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              std::vector<T> &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_INFO_STREAM("Missing parameter : " << param_name);
    return false;
  }
  data.reserve(YamlNode[param_name].size());
  for (int i = 0; i < YamlNode[param_name].size(); ++i) {
    data.push_back(static_cast<T>(YamlNode[param_name][i]));
  }
  return true;
}

bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              Eigen::VectorXd &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_INFO_STREAM("Missing parameter : " << param_name);
    return false;
  }
  data.resize(YamlNode[param_name].size());
  for (int i = 0; i < YamlNode[param_name].size(); ++i) {
    data(i) = static_cast<double>(YamlNode[param_name][i]);
  }
  return true;
}

}  // namespace utils
}  // namespace sampling
