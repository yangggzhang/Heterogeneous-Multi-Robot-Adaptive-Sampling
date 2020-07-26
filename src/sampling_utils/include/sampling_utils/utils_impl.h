#include "utils.h"

namespace sampling {
namespace utils {

template <typename T>
bool ExtractVec(const T &full, const std::vector<int> &ind, T &target) {
  int num_indices = ind.size();
  target = T(num_indices);
  for (int i = 0; i < num_indices; i++) {
    if (ind[i] >= full.size()) return false;
    target[i] = full[ind[i]];
  }
  return true;
}

template <typename T>
bool ExtractCols(const T &full, const std::vector<int> &ind, T &target) {
  target = T(full.rows(), ind.size());
  for (int i = 0; i < ind.size(); ++i) {
    if (ind[i] >= full.cols()) return false;
    target.col(i) = full.col(ind[i]);
  }
  return true;
}

template <typename T>
bool ExtractRows(const T &full, const std::vector<int> &ind, T &target) {
  target = T(ind.size(), full.cols());
  for (int i = 0; i < ind.size(); ++i) {
    if (ind[i] >= full.rows()) return false;
    target.row(i) = full.row(ind[i]);
  }
  return true;
}

template <typename T>
std::vector<T> Extract(const std::vector<T> &full,
                       const std::vector<int> &ind) {
  std::vector<T> target;
  target.reserve(ind.size());
  for (const T &i : ind) {
    target.push_back(full[i]);
  }
  return target;
}

template <typename T>
bool GetParam(const XmlRpc::XmlRpcValue &YamlNode,
              const std::string &param_name, T &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_WARN_STREAM("Missing parameter : " << param_name);
    return false;
  } else {
    data = static_cast<T>(YamlNode[param_name]);
  }
  return true;
}

template <typename T>
bool GetParam(const XmlRpc::XmlRpcValue &YamlNode,
              const std::string &param_name, std::vector<T> &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_WARN_STREAM("Missing parameter : " << param_name);
    return false;
  }
  data.reserve(YamlNode[param_name].size());
  for (int i = 0; i < YamlNode[param_name].size(); ++i) {
    data.push_back(static_cast<T>(YamlNode[param_name][i]));
  }
  return true;
}
}  // namespace utils
}  // namespace sampling
