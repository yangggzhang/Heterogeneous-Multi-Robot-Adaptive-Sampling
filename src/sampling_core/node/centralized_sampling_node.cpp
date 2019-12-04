#include <ros/ros.h>

#include "sampling_core/sampling_core.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  sampling::core::SamplingCore node(nh, rh);
  ROS_INFO_STREAM("finish initialzation");
  if (!node.Init()) {
    ROS_INFO_STREAM("Failed to initialize sampling core!");
    return -1;
  }
  ros::Rate r(60);  // 10 hz
  while (ros::ok()) {
    node.Update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
