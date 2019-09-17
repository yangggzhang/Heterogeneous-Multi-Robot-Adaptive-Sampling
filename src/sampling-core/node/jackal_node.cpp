#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
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
    return true;
  }

private:
  ros::NodeHandle nh_, rh_;
  std::string Jackal_movebase_channel_;
  std::string Jackal_movebase_frame_id_;
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
