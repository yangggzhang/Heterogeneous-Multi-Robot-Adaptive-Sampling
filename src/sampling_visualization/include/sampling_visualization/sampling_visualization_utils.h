#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <unordered_map>
#include <vector>

namespace sampling {
namespace visualization {

const double KHSVRange = 100.0;
const double KPixelScale = 255.0;

const int KRGBRedId = 0;
const std::vector<double> KRGBRed{255.0 / KPixelScale, 0.0 / KPixelScale,
                                  0.0 / KPixelScale};

const int KRGBGreenId = 1;
const std::vector<double> KRGBGreen{0.0 / KPixelScale, 255.0 / KPixelScale,
                                    0 / KPixelScale};

const int KRGBBlueId = 2;
const std::vector<double> KRGBBlue{0.0 / KPixelScale, 0 / KPixelScale,
                                   255 / KPixelScale};

const int KRGBYellowId = 3;
const std::vector<double> KRGBYellow{255.0 / KPixelScale, 255.0 / KPixelScale,
                                     0.0 / KPixelScale};

const int KRGBGreyId = 4;
const std::vector<double> KRGBGrey{128.0 / KPixelScale, 128.0 / KPixelScale,
                                   128.0 / KPixelScale};

const int KRGBPinkId = 5;
const std::vector<double> KRGBPink{255.0 / KPixelScale, 102.0 / KPixelScale,
                                   255.0 / KPixelScale};

class SamplingVisualizationUtils {
 public:
  SamplingVisualizationUtils();

  void HSVtoRGB(const double &H, const double &S, const double &V, double &R,
                double &G, double &B);

  bool UpdateColor(const double &norm, std_msgs::ColorRGBA &color);

  bool UpdateColor(const int &agent_id, std_msgs::ColorRGBA &color);

 private:
  std::unordered_map<int, std::vector<double>> color_map_;
};

}  // namespace visualization
}  // namespace sampling
