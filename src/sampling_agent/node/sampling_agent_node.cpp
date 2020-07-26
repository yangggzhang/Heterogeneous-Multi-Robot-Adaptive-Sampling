#include <string>

#include "sampling_agent/sampling_agent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sampling_agent_node");
  ros::NodeHandle nh, ph("~");
  ros::Rate r(2);

  std::unique_ptr<sampling::agent::SamplingAgent> agent =
      sampling::agent::SamplingAgent::MakeUniqueFromROS(nh, ph);

  if (agent == nullptr) {
    ROS_ERROR("Failed to create a sampling agent!");
    return -1;
  }

  // ros::AsyncSpinner spinner(0);
  // spinner.start();

  while (ros::ok()) {
    agent->Run();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}