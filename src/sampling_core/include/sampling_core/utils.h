
/**
 * Utility functions for sampling core
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <Eigen/Dense>
#include <string>

namespace sampling {
namespace utils {

bool LoadData(const std::string &path, Eigen::MatrixXd &data);

bool GetParamData(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
                  Eigen::MatrixXd &data);

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              T &data);

template <typename T>
bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              std::vector<T> &data);

bool GetParam(XmlRpc::XmlRpcValue &YamlNode, const std::string &param_name,
              Eigen::VectorXd &data);

void MsgToMatrix(const sampling_msgs::measurement &msg,
                 Eigen::MatrixXd &location, Eigen::MatrixXd &feature);

}  // namespace utils
}  // namespace sampling
#include "utils_impl.h"