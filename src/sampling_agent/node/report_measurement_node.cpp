#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sampling_msgs/measurement.h>
#include <sampling_msgs/temperature_measurement.h>
#include <sensor_msgs/NavSatFix.h>

namespace sampling {
namespace report {

using MEASUREMENT_SYNC = message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
        sensor_msgs::NavSatFix, sampling_msgs::temperature_measurement>>;

class MeasurementReporterNode {
 public:
  MeasurementReporterNode(const ros::NodeHandle &nh, const ros::NodeHandle &ph)
      : nh_(nh), ph_(ph) {
    std::string gps_channel, temperature_channel, report_channel;
    if (!ph_.getParam("gps_channel", gps_channel)) {
      ROS_INFO_STREAM("Error! Missing GPS channel!");
    }

    if (!ph_.getParam("temperature_channel", temperature_channel)) {
      ROS_INFO_STREAM("Error! Missing temperature channel!");
    }

    if (!ph_.getParam("report_channel", report_channel)) {
      ROS_INFO_STREAM("Error! Missing report channel!");
    }

    gps_sub_.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(
        nh_, gps_channel, 1));

    temperature_sub_.reset(
        new message_filters::Subscriber<sampling_msgs::temperature_measurement>(
            nh_, temperature_channel, 1));

    measurement_pub_ =
        nh_.advertise<sampling_msgs::measurement>(report_channel, 1);

    measurement_synchronizer_.reset(new MEASUREMENT_SYNC(
        MEASUREMENT_SYNC(60), *gps_sub_, *temperature_sub_));

    measurement_synchronizer_->registerCallback(
        boost::bind(&MeasurementReporterNode::MeasurementSynchronizerCallback,
                    this, _1, _2));
    ROS_INFO_STREAM("Finish initializing measurement reporter node!");
  }

  void MeasurementSynchronizerCallback(
      const sensor_msgs::NavSatFixConstPtr &gps_msg,
      const sampling_msgs::temperature_measurementConstPtr &temperature_msg) {
    sampling_msgs::measurement msg;
    msg.robot_id = 0;
    msg.valid = true;
    msg.location_x = gps_msg->latitude;
    msg.location_y = gps_msg->longitude;
    msg.measurement = temperature_msg->filtered_temperature;
    ROS_INFO_STREAM("Publishing!");
    measurement_pub_.publish(msg);
  }

 private:
  ros::NodeHandle nh_, ph_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> gps_sub_;

  std::unique_ptr<
      message_filters::Subscriber<sampling_msgs::temperature_measurement>>
      temperature_sub_;

  std::unique_ptr<MEASUREMENT_SYNC> measurement_synchronizer_;

  ros::Publisher measurement_pub_;
};
}  // namespace report
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "reporter_node");
  ros::NodeHandle nh, ph("~");
  sampling::report::MeasurementReporterNode node(nh, ph);
  ros::spin();
  return 0;
}