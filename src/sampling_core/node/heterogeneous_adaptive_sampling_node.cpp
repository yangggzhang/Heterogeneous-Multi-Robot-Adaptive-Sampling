#include <ros/ros.h>

#include "sampling_core/sampling_core.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "heteregoneous_adaptive_sampling");
  ros::NodeHandle nh, ph("~");
  std::unique_ptr<sampling::core::SamplingCore> sampling_core =
      sampling::core::SamplingCore::MakeUniqueFromRos(nh, ph);
  if (sampling_core == nullptr) {
    ROS_ERROR_STREAM("Failed to launch heterogeneous adaptive sampling!");
    return -1;
  }

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    sampling_core->Loop();
    loop_rate.sleep();
  }
  return 0;
}
