#include <ros/package.h>
#include <ros/ros.h>
#include <stdlib.h> /* srand, rand */
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

#include "sampling_core/utils.h"
#include "sampling_core/voronoi.h"
#include "sampling_core/voronoi_visualization.h"

float RandomFloat(float a, float b) {
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "voronoi_test_node");
  ros::NodeHandle nh("~");
  ros::Publisher voronoi_visualization_pub;
  voronoi_visualization_pub =
      nh.advertise<visualization_msgs::MarkerArray>("voronoi_visualization", 1);

  int num_robot;
  if (!nh.getParam("num_robot", num_robot)) {
    num_robot = 3;
  }
  std::vector<double> x_range, y_range;
  double resolution;
  if (!nh.getParam("x_range", x_range)) {
    x_range = {0, 100};
  }
  if (!nh.getParam("y_range", y_range)) {
    y_range = {0, 100};
  }
  if (!nh.getParam("resolution", resolution)) {
    resolution = 1.0;
  }
  const int num_x = int((x_range.back() - x_range.front()) / resolution);
  const int num_y = int((y_range.back() - y_range.front()) / resolution);

  Eigen::MatrixXd map(num_x * num_y, 2);
  int count = 0;
  for (int i = 0; i < num_x; ++i) {
    for (int j = 0; j < num_y; ++j) {
      map(count, 0) = x_range.front() + i * resolution;
      map(count, 1) = y_range.front() + j * resolution;
      count++;
    }
  }
  // voronoi
  // Load Voronoi Computation parameters
  std::vector<int> hetero_index;
  std::vector<sampling::HeterogenitySpace> hetero_spaces;

  if (!nh.getParam("hetero_index", hetero_index)) {
    ROS_ERROR_STREAM("Failed to heteregeneous state space!");
    return -1;
  }
  for (int i = 0; i < hetero_index.size(); ++i) {
    switch (hetero_index[i]) {
      case 0: {
        hetero_spaces.push_back(sampling::HeterogenitySpace::DISTANCE);
        break;
      }
      case 1: {
        hetero_spaces.push_back(sampling::HeterogenitySpace::SPEED);
        break;
      }
      case 2: {
        hetero_spaces.push_back(sampling::HeterogenitySpace::BATTERYLIFE);
        break;
      }
      case 3: {
        hetero_spaces.push_back(sampling::HeterogenitySpace::MOBILITY);
        break;
      }
      default: {
        ROS_ERROR_STREAM("Undefined heterogeneous space!");
        return -1;
      }
    }
  }

  std::vector<double> scale_factor;
  if (!nh.getParam("scale_factor", scale_factor)) {
    ROS_ERROR_STREAM("Failed to load scale factor!");
    return -1;
  }
  std::vector<std::vector<double>> motion_primitives;
  XmlRpc::XmlRpcValue motion_primitive_list;
  nh.getParam("motion_primitives", motion_primitive_list);
  assert(motion_primitive_list.size() == num_robot);
  for (int32_t i = 0; i < motion_primitive_list.size(); ++i) {
    XmlRpc::XmlRpcValue mp = motion_primitive_list[i];
    std::vector<double> param;
    if (!sampling::utils::GetParam(mp, "param", param)) {
      ROS_ERROR_STREAM("Failed to load motion primitive param " << i);
      return -1;
    }
    motion_primitives.push_back(param);
  }

  sampling::voronoi::Voronoi voronoi_map(map, num_robot, hetero_spaces,
                                         scale_factor, motion_primitives);

  Eigen::MatrixXd robot_locations(num_robot, 2);
  for (int i = 0; i < num_robot; i++) {
    robot_locations(i, 0) = RandomFloat(x_range.front(), x_range.back());
    robot_locations(i, 1) = RandomFloat(y_range.front(), y_range.back());
  }

  std::vector<int> cell_index = voronoi_map.GetVoronoiIndex(robot_locations);

  sampling::visualization::VoronoiVisualization voronoi_viz(map);
  voronoi_viz.UpdateMap(cell_index);
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(voronoi_viz.GetVoronoiMap());
  marker_array.markers.push_back(voronoi_viz.GetRobotMarker(robot_locations));
  ros::Rate r(60);  // 10 hz
  while (ros::ok()) {
    voronoi_visualization_pub.publish(marker_array);
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}
