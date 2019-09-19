#include "robot_agent/pelican_agent.h"

namespace sampling {
namespace agent {
PelicanNode::PelicanNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : AgentNode(nh, rh) {
  /// todo \paul \yunfei
  /// load necessary parameters

  cmd_pub = nh_.advertise<std_msgs::String>("pelican_command", 1); 

  tmp_sub = nh.subscribe("uav/temp", 1, tmp_callback); 

  gps_sub = nh.subscribe("fcu/gps", 1, PelicanNode::update_GPS_location_callback); 

  last_cmd_latitude = 0;
  last_cmd_longitude = 0;
  gps_converg_flag = true;

  wp_cmd = "launch_waypoint";
  pelican_wp_cmd.data = wp_cmd;
  cmd_pub.publish(ros_wp_cmd); 

  ROS_INFO_STREAM("Launching Pelican waypoint navigation.");
}

bool PelicanNode::update_goal_from_gps() {
  /// todo \paul \yunfei
  /// transform gps signal from rtk frame to pelican local frame
  /*TODO: Frame Transformation*/
  cmd_latitude = goal_rtk_latitude_;
  cmd_longitude = goal_rtk_longitude_;

  return true;
};

bool PelicanNode::navigate() {
  /// todo \paul \yunfei
  /// GPS waypoint navigation
  wp_cmd = "waypoint_height_auto," + std::to_string(last_cmd_latitude) + "," + std::to_string(last_cmd_longitude) + ",5000";
  ros::Duration(5).sleep();

  wp_cmd = "waypoint_height_auto," + std::to_string(cmd_latitude) + "," + std::to_string(cmd_longitude) + ",5000";

  gps_converg_flag = false;

  while(ros::ok() && !gps_converg_flag){
    ros::spinOnce(); 
    loop_rate.sleep(); 
  }

  last_cmd_latitude = cmd_latitude;
  last_cmd_longitude = cmd_longitude;

  wp_cmd = "waypoint_height_auto," + std::to_string(cmd_latitude) + "," + std::to_string(cmd_longitude) + ",1000";
  ros::Duration(5).sleep();

  return true;
}

void PelicanNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  /// todo \paul \yunfei
  /// update the local gps signal back to rtk frame
  double newlatitude = msg->latitude*pow(10,7);
  double newlongitude = msg->longitude*pow(10,7);

  /*TODO: Frame Transformation*/
  current_latitude_ = newlatitude;
  current_longitude_ = newlongitude;

  if(!gps_converg_flag){
    // ROS_INFO("I am in GPS gps_callback");
    lat_buff.push_back(newlatitude);
    long_buff.push_back(newlongitude);
    
    if(lat_buff.size()>5){
      lat_buff.erase(lat_buff.begin());
      long_buff.erase(long_buff.begin());
      bool dist_flag = true;       //flag to calculate the distance threshold
      double distance = 0;
      for(int i = 0; i < lat_buff.size()-1;++i){
        distance = sqrt(pow(lat_buff[i+1]-lat_buff[i],2)+pow(long_buff[i+1]-long_buff[i],2));
        if(distance>1500)       //Threshold = 1.5m
        {
          dist_flag = false;
          break;
        }
      }

      if(dist_flag){                //GPS converge, start temperature sampling
        ROS_INFO("GPS converge, start temperature sampling.");
        ROS_INFO_STREAM("Converged latitude: " <<std::setprecision(9)<< msg->latitude <<" Converged longitude: "<<msg->longitude<<"\n");
        gps_converg_flag = true;
      }
    }
  }
}

} // namespace agent
} // namespace sampling