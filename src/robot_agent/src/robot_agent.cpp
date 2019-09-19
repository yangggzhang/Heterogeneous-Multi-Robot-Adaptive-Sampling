#include "robot_agent/robot_agent.h"
#include <robot_agent/RequestGoal.h>
#include <robot_agent/RequestTemperatureMeasurement.h>

namespace sampling {
namespace agent {
AgentNode::AgentNode(){};

AgentNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : nh_(nh), rh_(rh) {

  if (!rh_.getParam("ros_queue_size", ros_queue_size_)) {
    ROS_ERROR("Error! Missing default ros queue size!");
  }

  if (!rh_.getParam("agent_id", agent_id_)) {
    ROS_ERROR("Error! Missing robot agent id!");
  }

  if (!rh_.getParam("request_target_channel", request_target_channel_)) {
    ROS_ERROR("Error! Missing robot agent request target channel!");
  }

  if (!rh_.getParam("temperature_measurement_channel",
                    temperature_measurement_channel_)) {
    ROS_ERROR("Error! Missing robot agent temperature measurement channel!");
  }

  if (!rh_.getParam("temperature_update_channel",
                    temperature_update_channel_)) {
    ROS_ERROR("Error! Missing robot agent temperature update channel!");
  }

  if (!rh_.getParam("gps_location_channel", gps_location_channel_)) {
    ROS_ERROR("Error! Missing robot agent gps location channel!");
  }

  agent_state_ = utils::IDLE;
  request_target_client_ =
      nh_.serviceClient<robot_agent::RequestGoal>(request_target_channel_);

  temperature_measurement_client_ =
      nh_.serviceClient<robot_agent::RequestTemperatureMeasurement>(
          temperature_measurement_channel_);

  temperature_sample_pub_ = nh_.advertise<sampling_core::measurement>(
      temperature_update_channel_, ros_queue_size_);

  gps_location_sub_ =
      nh_.subscribe(gps_location_channel_, ros_queue_size_,
                    &AgentNode::update_GPS_location_callback, this);
};
};
} // namespace agent
} // namespace sampling