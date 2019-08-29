#pragma once
namespace sampling {

// this function gets the color for each pixel given the normalized value of the
// pixel
void CentralizedController::getHeatMapColor(float norm, float &r, float &g,
                                            float &b) {
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
void rb_visualization(Eigen::MatrixXd Fss, Eigen::VectorXd pred_h,
                      Eigen::VectorXd pred_Var) {

  // initializing stuff for the marker msg
  visualization_msgs::Marker seed_point, heat_map_pred, heat_map_truth,
      heat_map_Var, color_bar, color_label, goal_position, robo_label,
      goal_label;
  this->vor_lines.header.frame_id = seed_point.header.frame_id =
      heat_map_pred.header.frame_id = heat_map_Var.header.frame_id =
          heat_map_truth.header.frame_id = color_bar.header.frame_id =
              color_label.header.frame_id = goal_position.header.frame_id =
                  robo_label.header.frame_id = goal_label.header.frame_id =
                      "/map";
  this->vor_lines.header.stamp = seed_point.header.stamp =
      heat_map_pred.header.stamp = heat_map_Var.header.stamp =
          heat_map_truth.header.stamp = color_bar.header.stamp =
              color_label.header.stamp = goal_position.header.stamp =
                  robo_label.header.stamp = goal_label.header.stamp =
                      ros::Time::now();
  this->vor_lines.ns = seed_point.ns = heat_map_pred.ns = heat_map_Var.ns =
      heat_map_truth.ns = color_bar.ns = color_label.ns = goal_position.ns =
          robo_label.ns = goal_label.ns = "testVor";
  this->vor_lines.pose.orientation.w = seed_point.pose.orientation.w =
      heat_map_pred.pose.orientation.w = heat_map_Var.pose.orientation.w =
          heat_map_truth.pose.orientation.w = color_bar.pose.orientation.w =
              color_label.pose.orientation.w =
                  goal_position.pose.orientation.w =
                      robo_label.pose.orientation.w =
                          goal_label.pose.orientation.w = 0.0;
  this->vor_lines.action = seed_point.action = heat_map_pred.action =
      heat_map_Var.action = heat_map_truth.action = color_bar.action =
          color_label.action = goal_position.action = robo_label.action =
              goal_label.action = visualization_msgs::Marker::ADD;

  // to keep the points from accidentally interfering with one to the other
  this->vor_lines.id = 10;
  seed_point.id = 11;
  heat_map_pred.id = 12;
  heat_map_truth.id = 13;
  heat_map_Var.id = 16;
  color_bar.id = 14;
  color_label.id = 15;
  // goal_position.id = 16;
  // robo_label.id = 17;
  // goal_label.id = 18;

  this->vor_lines.type = visualization_msgs::Marker::LINE_LIST;
  seed_point.type = visualization_msgs::Marker::SPHERE_LIST;
  heat_map_pred.type = visualization_msgs::Marker::CUBE_LIST;
  heat_map_Var.type = visualization_msgs::Marker::CUBE_LIST;
  heat_map_truth.type = visualization_msgs::Marker::CUBE_LIST;
  color_bar.type = visualization_msgs::Marker::CUBE_LIST;
  color_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // goal_position.type = visualization_msgs::Marker::CUBE_LIST;
  // robo_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // goal_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // only used to scale the width of the lines shown
  this->vor_lines.scale.x = 0.1;

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

  // goal_position.scale.x = 1.0;
  // goal_position.scale.y = 1.0;
  // goal_position.scale.z = 0.1;

  color_bar.scale.x = 1.0;
  color_bar.scale.y = 1.0;
  color_bar.scale.z = 0.1;

  color_label.scale.z = 1.0;

  // the r and b color values are automagically set to 0. That's why we need to
  // specify 'a' as 1.0 otherwise we won't be able to see anything in rviz
  seed_point.color.r = 0.0;
  seed_point.color.g = 0.0;
  seed_point.color.b = 0.0;
  seed_point.color.a = 1.0;

  color_label.color.r = 1.0;
  color_label.color.g = 1.0;
  color_label.color.b = 1.0;
  color_label.color.a = 1.0;

  color_label.pose.position.x = 5;
  color_label.pose.position.y = 49;
  color_label.pose.position.z = 0;

  // TODO:  add a for loop to loop over bots

  //????????????????????
  // Eigen::VectorXi idx_ = get_label_idx(this->k, robo_id);
  double max = Fss.maxCoeff();
  double min = Fss.minCoeff();

  double max_p = pred_h.maxCoeff(); // 12.9;
  double min_p = pred_h.minCoeff(); // 4.9;

  double max_v = pred_Var.maxCoeff(); // 12.9;
  double min_v = pred_Var.minCoeff(); // 4.9;

  // using the temperature data from Fss create the heat map visuals. Red are
  // hot spots with blue being cold spots
  int count = 0;
  bool raise = false;
  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 45; j++) {
      geometry_msgs::Point p;
      p.x = i;
      p.y = j * heat_map_pred.scale.y;
      // for(int k=0; k<idx_.size(); k++)
      // {
      //     if((i*45+j)==idx_(k))
      //     {
      //         raise = true;
      //         break;
      //     }
      // }
      if (raise == true) {
        p.z = 0.0;
        raise = false;
      } else {
        p.z = -1.0;
      }

      double norm;
      // cout<<pred_h(count)<<endl;
      if (isnan(pred_h(count))) {
        norm = 0;
        // cout<<"111111111111111111"<<endl;
      } else {
        if ((max - min) == 0 && max == 0) {
          norm = (pred_h(count) - min);
          // cout<<"2222222222222222222"<<endl;
        } else if ((max - min) == 0 && max != 0) {
          norm = (pred_h(count) - min) / max;
          // cout<<"3333333333333333333"<<endl;

        } else {
          norm = (pred_h(count) - min) / (max - min);
          // cout<<"44444444444444444444"<<endl;
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
      if (isnan(Fss(count, 0))) {
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
      if (isnan(pred_Var(count))) {
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

  // this->vor_pub.publish(this->vor_lines);
  this->vor_pub.publish(seed_point);
  this->vor_pub.publish(heat_map_pred);
  this->vor_pub.publish(heat_map_Var);
  this->vor_pub.publish(heat_map_truth);
  // this->vor_pub.publish(goal_position);
  // this->vor_pub.publish(robo_label);
  // this->vor_pub.publish(goal_label);

  // cout<<"7"<<endl;
}
}