#include <ros/ros.h>
#include <sampling_msgs/RequestTemperatureMeasurement.h>
#include <stdlib.h> /* srand, rand */
#include <string>

namespace sampling {
class TemperatureMeasurementSimulationNode {
public:
  TemperatureMeasurementSimulationNode(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {

    if (!rh_.getParam("temperature_report_service_channel",
                      temperature_report_service_channel_)) {
      ROS_ERROR("Error! Missing temperature report service channel!");
    }

    if (!rh_.getParam("max_temperature", max_temperature_)) {
      ROS_ERROR(
          "Error! Missing temperature measurement simulation upper bound!");
    }

    if (!rh_.getParam("min_temperature", min_temperature_)) {
      ROS_ERROR(
          "Error! Missing temperature measurement simulation lower bound!");
    }
    temperature_report_service_ = nh_.advertiseService(
        temperature_report_service_channel_,
        &TemperatureMeasurementSimulationNode::collect_temperature_sample,
        this);
  }

  bool collect_temperature_sample(
      sampling_msgs::RequestTemperatureMeasurement::Request &req,
      sampling_msgs::RequestTemperatureMeasurement::Response &res) {
    double random_temperature = (double)std::rand() / RAND_MAX;
    random_temperature =
        min_temperature_ +
        random_temperature * (max_temperature_ - min_temperature_);
    res.temperature = random_temperature;
    return true;
  }

private:
  ros::NodeHandle nh_, rh_;
  ros::ServiceServer temperature_report_service_;
  std::string temperature_report_service_channel_;
  double min_temperature_;
  double max_temperature_;
};
} // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "temperature_measurement_simulation");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::TemperatureMeasurementSimulationNode node(nh, rh);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
