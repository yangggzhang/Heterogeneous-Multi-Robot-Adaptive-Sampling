#include "robot_agent/jackal_agent.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "jackal_node");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::agent::JackalNode node(nh, rh);
  while (ros::ok()) {
    ROS_INFO_STREAM("JACKAL UPDATING!");
    node.collect_sample();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}