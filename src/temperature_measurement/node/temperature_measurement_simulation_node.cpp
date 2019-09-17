#include <ros/ros.h>
#include <stdlib.h> /* srand, rand */
#include <string>
#include <temperature_measurement/RequestTemperatureMeasurement.h>

namespace sampling {
class TemperatureMeasurementSimulationNode {
public:
  TemperatureMeasurementSimulationNode(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {}

  bool load_parameter() {
    if (!rh_.getParam("max_temperature", max_temperature_)) {
      ROS_INFO_STREAM(
          "Error! Missing temperature measurement simulation upper bound!");
      return false;
    }

    if (!rh_.getParam("min_temperature", min_temperature_)) {
      ROS_INFO_STREAM(
          "Error! Missing temperature measurement simulation lower bound!");
      return false;
    }

    ROS_INFO_STREAM("Finish loading data!");
    return true;
  }

  bool collect_temperature_sample(
      temperature_measurement::RequestTemperatureMeasurement::Request &req,
      temperature_measurement::RequestTemperatureMeasurement::Response &res) {
    double random_temperature = (double)std::rand() / RAND_MAX;
    random_temperature =
        min_temperature_ +
        random_temperature * (max_temperature_ - min_temperature_);
    res.temperature = random_temperature;
    return true;
  }

private:
  ros::NodeHandle nh_, rh_;
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
