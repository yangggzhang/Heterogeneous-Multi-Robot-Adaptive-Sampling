
/**
 * Utility functions for sampling project
 * AUTHOR: Yang Zhang
 */

#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

namespace sampling {
namespace utils {

template <typename T>
bool ExtractVec(const T &full, const std::vector<int> &ind, T &target);

template <typename T2>
bool ExtractCols(const T2 &full, const std::vector<int> &ind, T2 &target);

template <typename T2>
bool ExtractRows(const T2 &full, const std::vector<int> &ind, T2 &target);

template <typename T>
std::vector<T> Extract(const std::vector<T> &full, const std::vector<int> &ind);

template <typename T>
bool GetParam(const XmlRpc::XmlRpcValue &YamlNode,
              const std::string &param_name, T &data);

template <typename T>
bool GetParam(const XmlRpc::XmlRpcValue &YamlNode,
              const std::string &param_name, std::vector<T> &data);

}  // namespace utils
}  // namespace sampling
#include "sampling_utils/utils_impl.h"