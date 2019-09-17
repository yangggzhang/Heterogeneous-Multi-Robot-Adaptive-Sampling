#include "sampling_core/utils.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sampling_core/RequestGoal.h>
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
    request_target_client_ =
        nh_.serviceClient<sampling_core::RequestGoal>(request_target_channel_);
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
    return true;
  }

  bool request_target() {
    sampling_core::RequestGoal srv;
    srv.request.robot_id = robot_id_;
    srv.request.robot_latitude = current_location_.latitude;
    srv.request.robot_longitude = current_location_.longitude;

    if (request_target_client_.call(srv)) {
      ROS_INFO_STREAM("Robot " << robot_id_ << " received new target : ");
      ROS_INFO_STREAM("Latitude : " << srv.response.latitude << " Longitude : "
                                    << srv.response.longitude);
      gps_target_.latitude = srv.response.latitude;
      gps_target_.longitude = srv.response.longitude;
      return true;
    } else {
      ROS_ERROR("Robot " << robot_id
                         << " failed to request target from master computer!");
      return false;
    }
  }

private:
  ros::NodeHandle nh_, rh_;
  ros::ServiceClient request_target_client_;
  std::string request_target_channel_;
  std::string Jackal_movebase_channel_;
  std::string Jackal_movebase_frame_id_;

  std::string robot_id_;

  utils::gps_location current_location_;
  utils::gps_location gps_target_;
  utils::map_lcation map_target_;

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
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
