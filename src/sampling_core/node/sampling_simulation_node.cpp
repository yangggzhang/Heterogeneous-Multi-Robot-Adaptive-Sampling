#include <ros/ros.h>

#include "sampling_core/sampling_core_simulation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling_simulation");
  ros::NodeHandle nh, ph("~");
  sampling::core::SamplingCoreSimulation node(nh, ph);
  if (!node.Initialize()) {
    ROS_INFO_STREAM("Failed to initialize sampling core!");
    return -1;
  }
  ros::Rate r(2);
  while (ros::ok()) {
    node.Update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
