#pragma once
#include <math.h> /* Mod */
#include <ros/ros.h>
#include <stdlib.h>

#include "sampling_visualization/sampling_visualization_utils.h"

namespace sampling {
namespace visualization {

void HSVtoRGB(const double &H, const double &S, const double &V, double &R,
              double &G, double &B) {
  float C = V * S;  // Chroma
  float HPrime = fmod(H / 60.0, 6);
  float X = C * (1 - fabs(fmod(HPrime, 2) - 1));
  float M = V - C;

  if (0 <= HPrime && HPrime < 1) {
    R = C;
    G = X;
    B = 0;
  } else if (1 <= HPrime && HPrime < 2) {
    R = X;
    G = C;
    B = 0;
  } else if (2 <= HPrime && HPrime < 3) {
    R = 0;
    G = C;
    B = X;
  } else if (3 <= HPrime && HPrime < 4) {
    R = 0;
    G = X;
    B = C;
  } else if (4 <= HPrime && HPrime < 5) {
    R = X;
    G = 0;
    B = C;
  } else if (5 <= HPrime && HPrime < 6) {
    R = C;
    G = 0;
    B = X;
  } else {
    R = 0;
    G = 0;
    B = 0;
  }
  R += M;
  G += M;
  B += M;
}

bool UpdateColor(const double &norm, std_msgs::ColorRGBA &color) {
  if (norm < 0 || norm > 1) return false;
  double r, g, b;
  HSVtoRGB((1 - norm) * KHSVRange, 1.0, 1.0, r, g, b);
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0;
  return true;
}

bool UpdateColor(const int &agent_id, std_msgs::ColorRGBA &color) {
  if (agent_id < 0)
    return false;
  else if (!KColorMap.count(agent_id)) {
    srand(agent_id);
    double r = fmod((double)rand(), KPixelScale);
    double g = fmod((double)rand(), KPixelScale);
    double b = fmod((double)rand(), KPixelScale);
    KColorMap[agent_id] = std::vector<double>{r, g, b};
  }
  color.r = KColorMap[agent_id][0];
  color.g = KColorMap[agent_id][1];
  color.b = KColorMap[agent_id][2];
  color.a = 1.0;
  return true;
}
}  // namespace visualization
}  // namespace sampling
