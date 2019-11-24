#include <ros/ros.h>

#include "sampling_core/sampling_core.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "centralized_sampling");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::core::SamplingCore node(nh, rh);
  if (!node.Init()) {
    ROS_ERROR("Failed to initialize centralized sampling node!");
    return -1;
  }
  ros::spinOnce();

  while (ros::ok()) {
    node.Update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
