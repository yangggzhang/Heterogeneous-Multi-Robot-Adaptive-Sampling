#include "sampling_core/utils.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sampling_msgs/RequestGoal.h>
#include <sampling_msgs/RequestTemperatureMeasurement.h>
#include <sampling_msgs/measurement.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

namespace sampling {
class JackalNode {
public:
  JackalNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    if (!load_parameter()) {
      ROS_INFO_STREAM("Missing Jackal parameter!");
    };

    Jackal_action_client_ =
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            Jackal_movebase_channel_, true);
    while (!Jackal_action_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO_STREAM("Waiting for the move_base action server for "
                      << robot_id_ << " to come up");
    }
    request_target_client_ =
        nh_.serviceClient<sampling_msgs::RequestGoal>(request_target_channel_);

    temperature_measurement_client_ =
        nh_.serviceClient<sampling_msgs::RequestTemperatureMeasurement>(
            Jackal_temperature_measurement_channel_);

    temperature_sample_pub_ = nh_.advertise<sampling_msgs::measurement>(
        temperature_update_channel_, 10);

    gps_location_sub_ =
        nh_.subscribe(Jackal_GPS_channel_, 10,
                      &JackalNode::update_GPS_location_callback, this);

    Jackal_state_ = utils::IDLE;
  }

  bool load_parameter() {

    if (!rh_.getParam("Jackal_movebase_channel", Jackal_movebase_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal move base channel!");
      return false;
    }

    if (!rh_.getParam("Jackal_movebase_frame_id", Jackal_movebase_frame_id_)) {
      ROS_INFO_STREAM("Error! Missing Jackal move base goal frame!");
      return false;
    }

    if (!rh_.getParam("Jackal_request_target_channel",
                      request_target_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal target requesting channel!");
      return false;
    }

    if (!rh_.getParam("Jackal_temperature_measurement_channel",
                      Jackal_temperature_measurement_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal temperature measurement channel!");
      return false;
    }

    if (!rh_.getParam("Jackal_moving_duration_threshold_s",
                      Jackal_moving_duration_threshold_s_)) {
      ROS_INFO_STREAM("Error! Missing Jackal moving maximum duration!");
      return false;
    }

    if (!rh_.getParam("temperature_update_channel",
                      temperature_update_channel_)) {
      ROS_INFO_STREAM("Error! Missing temperature update channel!");
      return false;
    }

    return true;
  }

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg) {
    current_latitude_ = msg.latitude;
    current_longitude_ = msg.longitude;
  }

  bool request_target_from_master() {
    sampling_msgs::RequestGoal srv;
    srv.request.robot_id = robot_id_;
    srv.request.robot_latitude = current_location_.latitude;
    srv.request.robot_longitude = current_location_.longitude;

    if (request_target_client_.call(srv)) {
      gps_target_.latitude = srv.response.latitude;
      gps_target_.longitude = srv.response.longitude;
      return true;
    } else {
      ROS_INFO_STREAM("Robot "
                      << robot_id_
                      << " failed to request target from master computer!");
      return false;
    }
  }

  bool update_goal_from_gps() {
    /// todo \yang use utm to transform gps goal to map goal
    move_base_msgs::MoveBaseGoal empty_goal;
    move_base_goal_ = empty_goal;
    move_base_goal_.target_pose.header.frame_id = Jackal_movebase_frame_id_;
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.pose.position.x = gps_target_.latitude;
    move_base_goal_.target_pose.pose.position.y = gps_target_.longitude;

    /// todo \yang calculate
    move_base_goal_.target_pose.pose.orientation.w = 1.0;

    return true;
  }

  bool collect_temperature_sample() {
    sampling_msgs::RequestTemperatureMeasurement srv;
    srv.request.robot_id = robot_id_;

    if (temperature_measurement_client_.call(srv)) {
      temperature_measurement_ = srv.response.temperature;
      return true;
    } else {
      ROS_INFO_STREAM("Robot "
                      << robot_id_
                      << " failed to receive temperature measurement!");
      return false;
    }
  }

  bool navigate() {}

  void report_temperature_sample() {
    /// send temperature to maskter computer
    sampling_msgs::measurement msg;
    msg.valid = true;
    msg.latitude = current_latitude_;
    msg.longitude = current_longitude_;
    msg.measurement = temperature_measurement_;
    temperature_sample_pub_.publish(msg);
  }

  void collect_sample() {

    switch (Jackal_state_) {
    case utils::IDLE: {
      Jackal_state_ = utils::REQUEST;
      break;
    }
    case utils::REQUEST: {
      if (!request_target_from_master()) {
        ROS_INFO_STREAM("Robot : "
                        << robot_id_
                        << " failed to request target from master computer");
        ROS_INFO_STREAM("Retrying ... ... ...");
        break;
      } else {
        ROS_INFO_STREAM("Robot : " << robot_id_ << " succeeded in receiving "
                                                   "target from master "
                                                   "computer : ");
        ROS_INFO_STREAM("Latitude : " << gps_target_.latitude << " Longitude : "
                                      << gps_target_.longitude);

        if (!update_goal_from_gps()) {
          ROS_INFO_STREAM(
              "Failed to update local goal from GPS target location !");
          /// todo \yang keeps requesting?
          break;
        }
        Jackal_state_ = utils::NAVIGATE;
      }
      break;
    }
    case utils::NAVIGATE: {
      /// Infinite timing allowance rn
      Jackal_action_client_->sendGoal(move_base_goal_);
      Jackal_action_client_->waitForResult();
      if (Jackal_action_client_->getState() ==
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Hooray, robot " << robot_id_
                                         << " reached the target location!");
        Jackal_state_ = utils::REPORT;
        break;
      } else {
        ROS_INFO_STREAM("Robot "
                        << robot_id_
                        << " failed to reach the target location with state "
                        << Jackal_action_client_->getState().toString());
        Jackal_state_ = utils::REQUEST;
        break;
      }
    }
    case utils::REPORT: {
      if (!collect_temperature_sample()) {
        ROS_INFO_STREAM("Robot : " << robot_id_
                                   << " failed to measure temperature!");
        ROS_INFO_STREAM("Retrying ... ... ...");
      } else {
        ROS_INFO_STREAM("Robot " << robot_id_
                                 << " received new temperature measurement : "
                                 << temperature_measurement_);
        report_temperature_sample();
        Jackal_state_ = utils::REQUEST;
      }
    }
    default: {
      Jackal_state_ = utils::REQUEST;
      break;
    }
    }
  }

private:
  utils::STATE Jackal_state_;

  ros::NodeHandle nh_, rh_;
  ros::ServiceClient request_target_client_;
  ros::ServiceClient temperature_measurement_client_;
  ros::Publisher temperature_sample_pub_;
  ros::Subscriber gps_location_sub_;

  std::string robot_id_;
  double Jackal_moving_duration_threshold_s_;

  std::string request_target_channel_;
  std::string Jackal_movebase_channel_;
  std::string Jackal_movebase_frame_id_;
  std::string Jackal_temperature_measurement_channel_;
  std::string temperature_update_channel_;
  std::string Jackal_GPS_channel_;

  move_base_msgs::MoveBaseGoal move_base_goal_;
  double temperature_measurement_;
  double current_latitude_;
  double current_longitude_;

  utils::gps_location current_location_;
  utils::gps_location gps_target_;
  utils::map_location map_target_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      *Jackal_action_client_;
};
} // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "jackal_node");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::JackalNode node(nh, rh);
  while (ros::ok()) {
    node.collect_sample();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
