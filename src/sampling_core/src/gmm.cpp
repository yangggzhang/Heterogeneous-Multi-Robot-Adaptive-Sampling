#include "sampling_core/gmm.h"

#include <math.h>
#include <ros/ros.h>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

namespace sampling {
namespace gmm {
GaussianMixtureModel::GaussianMixtureModel() : cluster_number_(KClusterNumber) {
  model_ = cv::ml::EM::create();
  model_->setClustersNumber(KClusterNumber);
  model_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  model_->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, KMaxIteration, Keps));
  model_mean_.resize(cluster_number_);
  model_stdev_.resize(cluster_number_);
}

GaussianMixtureModel::GaussianMixtureModel(const int& cluster_number)
    : cluster_number_(cluster_number) {
  model_ = cv::ml::EM::create();
  model_->setClustersNumber(cluster_number);
  model_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  model_->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, KMaxIteration, Keps));
  model_mean_.resize(cluster_number_);
  model_stdev_.resize(cluster_number_);
}

GaussianMixtureModel::GaussianMixtureModel(const int& cluster_number,
                                           const int& max_iteration,
                                           const double& eps)
    : cluster_number_(cluster_number) {
  model_ = cv::ml::EM::create();
  model_->setClustersNumber(cluster_number);
  model_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  model_->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iteration, eps));
  model_mean_.resize(cluster_number_);
  model_stdev_.resize(cluster_number_);
}

bool GaussianMixtureModel::Train(const Eigen::VectorXd& samples) {
  cv::Mat samples_mat;
  cv::eigen2cv(samples, samples_mat);
  if (!model_->trainEM(samples_mat)) return false;
  cv::Mat cv_model_mean;
  std::vector<cv::Mat> cv_model_cov;
  cv_model_mean = model_->getMeans();
  model_->getCovs(cv_model_cov);
  for (int i = 0; i < cluster_number_; ++i) {
    model_mean_[i] = cv_model_mean.at<double>(i, 0);
    model_stdev_[i] = std::sqrt(cv_model_cov[i].at<double>(0, 0));
  }
}

Eigen::MatrixXd GaussianMixtureModel::Predict(const Eigen::VectorXd& samples) {
  Eigen::MatrixXd probs;
  cv::Mat samples_mat, probs_mat;
  cv::eigen2cv(samples, samples_mat);
  model_->predict(samples_mat, probs_mat);
  cv::cv2eigen(probs_mat, probs);
  return probs;
}

Eigen::MatrixXd GaussianMixtureModel::GetMeans() {
  Eigen::MatrixXd means_eigen;
  cv::Mat means_mat = model_->getMeans();
  cv::cv2eigen(means_mat, means_eigen);
  return means_eigen;
}

Eigen::VectorXd GaussianMixtureModel::ProbPredict(
    const int& cluster_id, const Eigen::VectorXd& samples) {
  Eigen::VectorXd prob;
  prob.resize(samples.size());
  for (int i = 0; i < prob.size(); ++i) {
    if (std::isinf(samples(i)) || std::isnan(samples(i))) {
      prob(i) = 0.0;
    } else {
      prob(i) = boost::math::pdf(nd, (samples(i) - model_mean_[cluster_id]) /
                                         model_stdev_[cluster_id]);
    }
  }
  return prob;
}

}  // namespace gmm
}  // namespace sampling