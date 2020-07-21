#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace sampling {
namespace visualization {

const std::string KVisualizationType_Grid = "GRID";
const std::string KVisualizationType_Location = "LOCATION";
const std::string KVisualizationType_Partition = "PARTITION";

const std::string KVisualizationNamespace = "/visualization/";

const double KVisualizationUpdateRate_hz = 1.0;

const size_t KVisualizationDimension = 2;
const double KPixelScale = 255.0;
const double KVisualizationUpperBound = 5.0;
const double KVisualizationLowerBound = -5.0;
const double KRGBRed[3] = {255.0 / KPixelScale, 0.0 / KPixelScale,
                           0.0 / KPixelScale};
const double KRGBGreen[3] = {0.0 / 255.0, 255 / 255.0, 0 / 255.0};
const double KRGBBlue[3] = {0.0 / 255.0, 0 / 255.0, 255 / 255.0};
const double KRGBYellow[3] = {255.0 / 255.0, 255.0 / 255.0, 0.0 / 255.0};
const double KRGBGray[3] = {128.0 / 255.0, 128.0 / 255.0, 128.0 / 255.0};
const double KRGBPink[3] = {255.0 / 255.0, 102.0 / 255.0, 255.0 / 255.0};

class SamplingVisualizationParams {
 public:
  SamplingVisualizationParams();

  bool LoadFromXML(const XmlRpc::XmlRpcValue& param);

  std::string name;

  std::string visualization_type;

  std::vector<double> offset;

  std::vector<double> scale;

  // lower bound and upper bound
  std::vector<double> bounds;
};

}  // namespace visualization
}  // namespace sampling