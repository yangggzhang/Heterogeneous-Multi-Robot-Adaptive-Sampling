#pragma once
#include "sampling-core/sampling_visualization.h"

namespace sampling {

// this function gets the color for each pixel given the normalized value of the
// pixel
void getHeatMapColor(float norm, float &r, float &g, float &b) {
  const int NUM_COLORS = 5;
  static float color[NUM_COLORS][3] = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 0}, {1, 0, 0}, {0.6, 0, 0}};
  int idx1, idx2;
  float fracB = 0;

  if (norm <= 0) {
    idx1 = idx2 = 0;
  } else if (norm >= 1) {
    idx1 = idx2 = NUM_COLORS - 1;
  } else {
    norm = norm * (NUM_COLORS - 1);
    idx1 = floor(norm);
    idx2 = idx1 + 1;
    fracB = norm - (float)idx1;
  }

  r = (color[idx2][0] - color[idx1][0]) * fracB + color[idx1][0];
  g = (color[idx2][1] - color[idx1][1]) * fracB + color[idx1][1];
  b = (color[idx2][2] - color[idx1][2]) * fracB + color[idx1][2];
}

// This function is used to generate the rviz visualization of the robot
// positions, corresponding voronoi edges, and heat map
void rb_visualization(const Eigen::MatrixXd& Fss,const Eigen::VectorXd& pred_h,
                      const Eigen::VectorXd& pred_Var, visualization_msgs::Marker& seed_point,
                      visualization_msgs::Marker& heat_map_pred, visualization_msgs::Marker& heat_map_Var,
                      visualization_msgs::Marker& heat_map_truth) {

  // initializing stuff for the marker msg
  visualization_msgs::Marker empty_marker;
  seed_point = heat_map_pred = heat_map_truth = heat_map_Var = empty_marker;

  seed_point.header.frame_id =
      heat_map_pred.header.frame_id = heat_map_Var.header.frame_id =
          heat_map_truth.header.frame_id =
                      "/map";
  seed_point.header.stamp =
      heat_map_pred.header.stamp = heat_map_Var.header.stamp =
          heat_map_truth.header.stamp =
                      ros::Time::now();
  seed_point.ns = heat_map_pred.ns = heat_map_Var.ns =
      heat_map_truth.ns = "testVor";
  seed_point.pose.orientation.w =
      heat_map_pred.pose.orientation.w = heat_map_Var.pose.orientation.w =
          heat_map_truth.pose.orientation.w = 0.0;
  seed_point.action = heat_map_pred.action =
      heat_map_Var.action = heat_map_truth.action = visualization_msgs::Marker::ADD;

  // to keep the points from accidentally interfering with one to the other
  seed_point.id = 11;
  heat_map_pred.id = 12;
  heat_map_truth.id = 13;
  heat_map_Var.id = 16;

  seed_point.type = visualization_msgs::Marker::SPHERE_LIST;
  heat_map_pred.type = visualization_msgs::Marker::CUBE_LIST;
  heat_map_Var.type = visualization_msgs::Marker::CUBE_LIST;
  heat_map_truth.type = visualization_msgs::Marker::CUBE_LIST;

  // points need both x and y scale
  seed_point.scale.x = 0.4;
  seed_point.scale.y = 0.4;
  seed_point.scale.z = 0.4;
  heat_map_pred.scale.x = 1.0;
  heat_map_pred.scale.y = 0.5;
  heat_map_pred.scale.z = 0.1;
  heat_map_truth.scale.x = 1.0;
  heat_map_truth.scale.y = 0.5;
  heat_map_truth.scale.z = 0.1;
  heat_map_Var.scale.x = 1.0;
  heat_map_Var.scale.y = 0.5;
  heat_map_Var.scale.z = 0.1;

  // the r and b color values are automagically set to 0. That's why we need to
  // specify 'a' as 1.0 otherwise we won't be able to see anything in rviz
  seed_point.color.r = 0.0;
  seed_point.color.g = 0.0;
  seed_point.color.b = 0.0;
  seed_point.color.a = 1.0;

  // TODO:  add a for loop to loop over bots

  // Eigen::VectorXi idx_ = get_label_idx(this->k, robo_id);
  double max = Fss.maxCoeff();
  double min = Fss.minCoeff();

  double max_p = pred_h.maxCoeff(); // 12.9;
  double min_p = pred_h.minCoeff(); // 4.9;

  double max_v = pred_Var.maxCoeff(); // 12.9;
  double min_v = pred_Var.minCoeff(); // 4.9;

  // using the temperature data from Fss create the heat map visuals. Red are
  int count = 0;
  bool raise = false;
  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 45; j++) {
      geometry_msgs::Point p;
      p.x = i;
      p.y = j * heat_map_pred.scale.y;
      if (raise == true) {
        p.z = 0.0;
        raise = false;
      } else {
        p.z = -1.0;
      }

      double norm;
      if (std::isnan(pred_h(count))) {
        norm = 0;
      } else {
        if ((max - min) == 0 && max == 0) {
          norm = (pred_h(count) - min);
        } else if ((max - min) == 0 && max != 0) {
          norm = (pred_h(count) - min) / max;
        } else {
          norm = (pred_h(count) - min) / (max - min);
        }
      }

      std_msgs::ColorRGBA c;
      float r, g, b;
      getHeatMapColor((float)norm, r, g, b);

      c.r = r;
      c.g = g;
      c.b = b;
      c.a = 1.0;

      heat_map_pred.points.push_back(p);
      heat_map_pred.colors.push_back(c);
      count++;
    }
  }

  count = 0;
  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 45; j++) {
      geometry_msgs::Point p;
      p.x = i - 25;
      p.y = j * heat_map_truth.scale.y;
      p.z = 0;
      double norm;
      if (std::isnan(Fss(count, 0))) {
        norm = 0;
      } else {
        if ((max - min) == 0 && max == 0) {
          norm = (Fss(count, 0) - min);
        } else if ((max - min) == 0 && max != 0) {
          norm = (Fss(count, 0) - min) / max;
        } else {
          norm = (Fss(count, 0) - min) / (max - min);
        }
      }
      std_msgs::ColorRGBA c;
      float r, g, b;
      getHeatMapColor((float)norm, r, g, b);

      c.r = r;
      c.g = g;
      c.b = b;
      c.a = 1.0;

      heat_map_truth.points.push_back(p);
      heat_map_truth.colors.push_back(c);
      count++;
    }
  }

  count = 0;
  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 45; j++) {
      geometry_msgs::Point p;
      p.x = i + 25;
      p.y = j * heat_map_Var.scale.y;
      p.z = 0;
      double norm;
      if (std::isnan(pred_Var(count))) {
        norm = 0;
      } else {
        if ((max - min) == 0 && max == 0) {
          norm = (pred_Var(count) - min);
        } else if ((max - min) == 0 && max != 0) {
          norm = (pred_Var(count) - min) / max;
        } else {
          norm = (pred_Var(count) - min) / (max - min);
        }
      }
      std_msgs::ColorRGBA c;
      float r, g, b;
      getHeatMapColor((float)norm, r, g, b);

      c.r = r;
      c.g = g;
      c.b = b;
      c.a = 1.0;

      heat_map_Var.points.push_back(p);
      heat_map_Var.colors.push_back(c);
      count++;
    }
  }
}
}