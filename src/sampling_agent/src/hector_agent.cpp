#include "sampling_agent/hector_agent.h"

#include <hector_navigation_msgs/Navigation.h>
#include <hector_navigation_msgs/Takeoff.h>

namespace sampling {
namespace agent {

HectorAgent::HectorAgent(ros::NodeHandle &nh, const std::string &agent_id,
                         const HectorAgentParam &params)
    : SamplingAgent(nh, agent_id), params_(params), taken_off_(false) {
  odom_subscriber_ = nh.subscribe(nh.getNamespace() + "/ground_truth/state", 1,
                                  &HectorAgent::UpdatePositionFromOdom, this);

  hector_takeoff_client_ = nh.serviceClient<hector_navigation_msgs::Takeoff>(
      nh.getNamespace() + "/hector_takeoff");

  hector_navigate_client_ =
      nh.serviceClient<hector_navigation_msgs::Navigation>(
          nh.getNamespace() + "/hector_navigation");
}

std::unique_ptr<HectorAgent> HectorAgent::MakeUniqueFromROSParam(
    ros::NodeHandle &nh) {
  std::string agent_id;
  nh.param<std::string>("agent_id", agent_id, "hector0");

  ros::NodeHandle ph("~");
  HectorAgentParam params;

  if (!params.LoadFromRosParams(ph)) {
    ROS_ERROR("Failed to load parameters for hector agent!");
    return nullptr;
  }

  return std::unique_ptr<HectorAgent>(new HectorAgent(nh, agent_id, params));
}

bool HectorAgent::Navigate() {
  if (!taken_off_) {
    hector_navigation_msgs::Takeoff take_off_srv;
    take_off_srv.request.takeoff_distance_m = params_.takeoff_distance_m;
    if (hector_takeoff_client_.call(take_off_srv)) {
      if (take_off_srv.response.success) {
        ROS_INFO_STREAM("Hector" << agent_id_ << "has taken off!");
        taken_off_ = true;
      } else {
        ROS_INFO_STREAM("Hector" << agent_id_ << " failed to take off!");
        return false;
      }
    } else {
      ROS_INFO_STREAM("Hector" << agent_id_
                               << " lost connect to take-off service!");
      return false;
    }
  }

  if (!target_position_.is_initialized()) {
    ROS_INFO_STREAM("Hector" << agent_id_
                             << " failed to receive sampling target!");
    return false;
  }
  hector_navigation_msgs::Navigation navigation_srv;
  navigation_srv.request.goal = target_position_.get();
  navigation_srv.request.goal.z = params_.navigation_height_m;
  navigation_srv.request.speed = params_.navigation_speed_ms;

  if (!hector_navigate_client_.call(navigation_srv)) {
    ROS_INFO_STREAM("Hector" << agent_id_
                             << " lost connect to navigation service!");
  } else {
    if (navigation_srv.response.return_type == 0) {
      ROS_INFO_STREAM("Hector" << agent_id_ << "succeeded in navigation!");
      return true;
    } else {
      ROS_INFO_STREAM("Hector" << agent_id_ << "failed navigation!");
      return false;
    }
  }
  return false;
}

void HectorAgent::UpdatePositionFromOdom(const nav_msgs::Odometry &msg) {
  current_position_ = boost::make_optional(msg.pose.pose.position);
}

}  // namespace agent
}  // namespace sampling
