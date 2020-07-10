#include "sampling_agent/fake_agent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_agent_node");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::agent::FakeSamplingAgent node(nh, rh);
  while (ros::ok()) {
    node.collect_sample();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}