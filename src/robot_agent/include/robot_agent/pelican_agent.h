#pragma once

#include "robot_agent/robot_agent.h"

/// todo \Paul \Yunfei
/// basic function of Pelican execution
/// reference robot_agent.h and jackal_agent.h

namespace sampling {
namespace agent {

class PelicanNode : public AgentNode {
public:
  PelicanNode(){};

  PelicanNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  bool update_goal_from_gps();

  bool navigate();

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg);

private:
	ros::Publisher cmd_pub; 
	ros::Subscriber tmp_sub; 
    ros::Subscriber gps_sub; 

    std_msgs::String pelican_wp_cmd;
    std::String wp_cmd;

    double cmd_latitude;
  	double cmd_longitude;
  	double last_cmd_latitude;
  	double last_cmd_longitude;

  	bool gps_converg_flag;      //flag to detect the converge of GPS sensor
};
} // namespace agent
} // namespace sampling