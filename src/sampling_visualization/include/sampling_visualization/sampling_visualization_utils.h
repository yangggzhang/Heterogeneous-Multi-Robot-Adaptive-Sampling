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
const std::vector<double> KRGBGreen{0.0 / 255.0, 255 / 255.0, 0 / 255.0};

const int KRGBBlueId = 2;
const std::vector<double> KRGBBlue{0.0 / 255.0, 0 / 255.0, 255 / 255.0};

const int KRGBYellowId = 3;
const std::vector<double> KRGBYellow{255.0 / 255.0, 255.0 / 255.0, 0.0 / 255.0};

const int KRGBGreyId = 4;
const std::vector<double> KRGBGrey{128.0 / 255.0, 128.0 / 255.0, 128.0 / 255.0};

const int KRGBPinkId = 5;
const std::vector<double> KRGBPink{255.0 / 255.0, 102.0 / 255.0, 255.0 / 255.0};

std::unordered_map<int, std::vector<double>> KColorMap{
    {KRGBRedId, KRGBRed},   {KRGBGreenId, KRGBGreen},
    {KRGBBlueId, KRGBBlue}, {KRGBYellowId, KRGBYellow},
    {KRGBGreyId, KRGBGrey}, {KRGBPinkId, KRGBPink}};

void HSVtoRGB(const double &H, const double &S, const double &V, double &R,
              double &G, double &B);

bool GetColor(const double &norm, std_msgs::ColorRGBA &color);

bool GetColor(const int &agent_id, std_msgs::ColorRGBA &color);
}  // namespace visualization
}  // namespace sampling
#include "sampling_visualization/sampling_visualization_utils_impl.h"