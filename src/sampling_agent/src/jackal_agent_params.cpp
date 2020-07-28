#include "sampling_agent/jackal_agent_params.h"

namespace sampling {
namespace agent {

JackalAgentParams::JackalAgentParams() {}

bool JackalAgentParams::LoadFromRosParams(ros::NodeHandle& ph) {
  ph.param<std::string>("navigation_frame", navigation_frame,
                        KJackalWorldFrame);

  std::string navigation_mode_str;

  ph.param<std::string>("navigation_mode", navigation_mode_str,
                        KJackalNavigationMode);
  if (navigation_mode_str.compare("ODOM") == 0) {
    navigation_mode = ODOM;
  } else if (navigation_mode_str.compare("GPS") == 0) {
    navigation_mode = GPS;
  } else {
    ROS_ERROR_STREAM("Unkown navigation mode for Jackal!");
    return false;
  }

  ph.param<double>("max_speed_ms", max_speed_ms, KJackalMaxSpeed_ms);

  ph.param<double>("execute_timeout_s", execute_timeout_s,
                   KJackalExecuteTimeout_s);

  ph.param<double>("preempt_timeout_s", preempt_timeout_s,
                   KJackalPreemptTimeout_s);
  return true;
}

}  // namespace agent
}  // namespace sampling