
/**
 * Utility functions for sampling core
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <Eigen/Dense>
#include <string>
#include "sampling_core/sampling_visualization.h"

namespace sampling {
namespace utils {

bool LoadData(const std::string &path, Eigen::MatrixXd &data);

bool LoadDataVec(const std::string &path, Eigen::VectorXd &data);

bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode,
                  sampling::visualization::MAP_PARAM &param);

bool GetParamData(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
                  Eigen::MatrixXd &data);

bool GetParamDataVec(XmlRpc::XmlRpcValue &YamlNode,
                     const std::string &param_name, Eigen::VectorXd &data);

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              T &data);

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              std::vector<T> &data);

bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              Eigen::VectorXd &data);

// bool GetMapParam(XmlRpc::XmlRpcValue &YamlNode, MAP_PARAM &param);

void MsgToMatrix(const sampling_msgs::measurement &msg,
                 Eigen::MatrixXd &location, Eigen::MatrixXd &feature);

inline double L2Distance(const Eigen::MatrixXd &location0,
                         const Eigen::MatrixXd &location1) {
  double dx = location0(0, 0) - location1(0, 0);
  double dy = location0(0, 1) - location1(0, 1);
  return dx * dx + dy * dy;
}

double CalculateRMS(Eigen::VectorXd &pred, Eigen::VectorXd &gt);

}  // namespace utils
}  // namespace sampling
#include "utils_impl.h"