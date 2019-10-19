#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

namespace sampling {
namespace visualization {
const int K_NUM_COLOR = 5;
const double K_COLOR[K_NUM_COLOR][3] = {
    {0, 0, 1}, {0, 1, 1}, {1, 1, 0}, {1, 0, 0}, {0.6, 0, 0}};

class sampling_visualization {
 public:
  sampling_visualization();

  sampling_visualization(const double &x_scale, const double &y_scale,
                         const double &z_scale, const int &latitude_range,
                         const int &longitude_range);

  void initialize_map(const std::string &visualization_frame,
                      const std::string &name_space, const int &map_id,
                      visualization_msgs::Marker &map);

  void update_map(const int &offset, const Eigen::VectorXd &filling_value,
                  visualization_msgs::Marker &map);

 private:
  int latitude_range_, longitude_range_;

  double x_scale_, y_scale_, z_scale_;

  // this function gets the color for each pixel given the normalized value of
  // the pixel
  void get_heatmap_color(const double &norm, std_msgs::ColorRGBA &color);
};
}  // namespace visualization
}  // namespace sampling