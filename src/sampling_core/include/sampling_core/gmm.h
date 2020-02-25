/**
 * Gaussian Mixture Model
 * AUTHOR: Yang Zhang
 */

#pragma once

#include <stdlib.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace sampling {
namespace GMM_CV {

const int KClusterNumber = 3;
const int KMaxIteration = 300;
const double Keps = 0.1;

class GaussianMixtureModel {
 private:
 public:
  GaussianMixtureModel();

  GaussianMixtureModel(const int& cluster_number, const int& max_iteration,
                       const double& eps);

  bool Train(const Eigen::VectorXd& samples);

  Eigen::MatrixXd Predict(const Eigen::VectorXd& samples);

  Eigen::MatrixXd GetMeans();

 private:
  cv::Ptr<cv::ml::EM> model_;
};
}  // namespace GMM_CV
}  // namespace sampling