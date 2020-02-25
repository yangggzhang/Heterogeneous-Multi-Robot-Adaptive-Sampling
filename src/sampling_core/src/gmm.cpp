#include "sampling_core/gmm.h"

#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

namespace sampling {
namespace GMM_CV {
GaussianMixtureModel::GaussianMixtureModel() {
  model_ = cv::ml::EM::create();
  model_->setClustersNumber(KClusterNumber);
  model_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  model_->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, KMaxIteration, Keps));
}

GaussianMixtureModel::GaussianMixtureModel(const int& cluster_number,
                                           const int& max_iteration,
                                           const double& eps) {
  model_ = cv::ml::EM::create();
  model_->setClustersNumber(cluster_number);
  model_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  model_->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iteration, eps));
}

bool GaussianMixtureModel::Train(const Eigen::VectorXd& samples) {
  cv::Mat samples_mat;
  cv::eigen2cv(samples, samples_mat);
  cv::Size s = samples_mat.size();
  return model_->trainEM(samples_mat);
}

void GaussianMixtureModel::Predict(const Eigen::VectorXd& samples,
                                   Eigen::MatrixXd& probs) {
  std::vector<double> sample_array;
  sample_array.resize(samples.size());
  Eigen::VectorXd::Map(&sample_array.front(), samples.size()) = samples;
  cv::Mat probs_mat;
  model_->predict(sample_array, probs_mat);
  cv::cv2eigen(probs_mat, probs);
  return;
}

Eigen::MatrixXd GaussianMixtureModel::GetMeans() {
  Eigen::MatrixXd means_eigen;
  cv::Mat means_mat = model_->getMeans();
  cv::cv2eigen(means_mat, means_eigen);
  return means_eigen;
}

}  // namespace GMM_CV
}  // namespace sampling