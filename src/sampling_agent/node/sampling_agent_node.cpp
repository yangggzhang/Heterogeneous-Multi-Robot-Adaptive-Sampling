#include <string>

#include "sampling_agent/sampling_agent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sampling_agent_node");
  ros::NodeHandle nh, ph("~");
  ros::Rate r(10);

  std::string agent_type;
  if (!ph.getParam("agent_type", agent_type)) {
    ROS_ERROR("Please specify sampling agent type!");
    return -1;
  }

  std::unique_ptr<sampling::agent::SamplingAgent> agent =
      sampling::agent::SamplingAgent::MakeUniqueFromROS(nh, agent_type);

  if (agent == nullptr) {
    ROS_ERROR("Failed to create a sampling agent!");
    return -1;
  }

  while (ros::ok()) {
    agent->Run();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}