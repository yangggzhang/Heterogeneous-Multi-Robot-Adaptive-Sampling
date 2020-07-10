#include "sampling_agent/pelican_agent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pelican_node");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::agent::PelicanAgent node(nh, rh);
  while (ros::ok()) {
    node.collect_sample();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}