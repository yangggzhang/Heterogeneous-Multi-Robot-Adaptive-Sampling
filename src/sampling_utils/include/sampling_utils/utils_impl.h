#include "utils.h"

namespace sampling {
namespace utils {

template <typename T>
bool GetParam(const XmlRpc::XmlRpcValue &YamlNode,
              const std::string &param_name, T &data) {
  if (!YamlNode.hasMember(param_name)) {
    ROS_INFO_STREAM("Missing parameter : " << param_name);
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
    ROS_INFO_STREAM("Missing parameter : " << param_name);
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
