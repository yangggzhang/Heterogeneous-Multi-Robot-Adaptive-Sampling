#include <assert.h>
#include <ros/ros.h>

#include "sampling_core/gpmm.h"

namespace sampling {
namespace gpmm {
GaussianProcessMixtureModel::GaussianProcessMixtureModel()
    : gp_number_(KGaussianProcessNumber) {
  gmm_model_ = std::unique_ptr<gmm::GaussianMixtureModel>(
      new gmm::GaussianMixtureModel(KGaussianProcessNumber));
  gp_model_.resize(KGaussianProcessNumber);
  for (int i = 0; i < gp_model_.size(); ++i) {
    gp_model_[i] =
        new libgp::GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)");
    Eigen::VectorXd params(KGPParamNumber);
    params << KGPCov1, KGPCov2, KGPNoise;
    gp_model_[i]->covf().set_loghyper(params);
  }
  // optimizer_.init();
}

GaussianProcessMixtureModel::GaussianProcessMixtureModel(
    const int& gp_number, const std::vector<std::vector<double>> gp_hyperparams,
    const int& gp_max_iteration, const double& gp_eps)
    : gp_number_(gp_number) {
  gmm_model_ = std::unique_ptr<gmm::GaussianMixtureModel>(
      new gmm::GaussianMixtureModel(gp_number, gp_max_iteration, gp_eps));
  assert(gp_number == gp_hyperparams.size());
  gp_model_.resize(gp_number);
  for (int i = 0; i < gp_number; ++i) {
    gp_model_[i] =
        new libgp::GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)");
    Eigen::VectorXd params(KGPParamNumber);
    assert(gp_hyperparams[i].size() == KGPParamNumber);
    params << gp_hyperparams[i][0], gp_hyperparams[i][1], gp_hyperparams[i][2];
    gp_model_[i]->covf().set_loghyper(params);
  }
  // optimizer_.init();
}

void GaussianProcessMixtureModel::Train(
    const Eigen::VectorXd& sample_utilities,
    const Eigen::MatrixXd& sample_positions) {
  const int sample_number = sample_utilities.size();

  // Gaussian Mixture Model Expectation and Maximation Training
  gmm_model_->Train(sample_utilities);
  // Gaussian Mixture Model Prediction
  Eigen::MatrixXd prob_matrix = gmm_model_->Predict(sample_utilities);
  // Add training pattern to Gaussian Process Mixture model
  for (int i = 0; i < gp_number_; ++i) {
    gp_model_[i]->clear_sampleset();
  }

  std::vector<int> assigned_sample(gp_number_, 0);
  for (int i = 0; i < sample_number; ++i) {
    Eigen::MatrixXd::Index predicted_class;
    double sample_utility = sample_utilities(i);
    double sample_position[] = {sample_positions(i, 0), sample_positions(i, 1)};
    prob_matrix.row(i).maxCoeff(&predicted_class);
    gp_model_[predicted_class]->add_pattern(sample_position, sample_utility);
    assigned_sample[predicted_class]++;
  }

  for (int i = 0; i < gp_number_; ++i) {
    if (assigned_sample[i] == 0) continue;
    optimizer_.maximize(gp_model_[i], KGPUpdateNum, 0);
  }
}

void GaussianProcessMixtureModel::GPPredict(const int& GP_index,
                                            const Eigen::MatrixXd& locations,
                                            Eigen::MatrixXd& pred_mean,
                                            Eigen::MatrixXd& pred_var) {
  for (int i = 0; i < locations.rows(); ++i) {
    double test_location[] = {locations(i, 0), locations(i, 1)};
    pred_mean(i, GP_index) = gp_model_[GP_index]->f(test_location);
    pred_var(i, GP_index) = gp_model_[GP_index]->var(test_location);
  }
}

Eigen::VectorXd GaussianProcessMixtureModel::GMMPredictForGP(
    const int& GP_index, const Eigen::VectorXd& gp_predictions) {
  return gmm_model_->ProbPredict(GP_index, gp_predictions);
}

void GaussianProcessMixtureModel::Predict(const Eigen::MatrixXd& test_positions,
                                          Eigen::VectorXd& pred_mean,
                                          Eigen::VectorXd& pred_var) {
  const int test_number = test_positions.rows();
  Eigen::MatrixXd raw_mean_predictions(test_number, gp_number_),
      raw_var_predictions(test_number, gp_number_);
  for (int i = 0; i < gp_number_; ++i) {
    GPPredict(i, test_positions, raw_mean_predictions, raw_var_predictions);
  }

  // N x num_gp matrix, matrix(i,j) is the normalized probability of sample_i
  // belongs to gp_j
  Eigen::MatrixXd prob_mat(test_positions.rows(), gp_number_);
  for (int i = 0; i < gp_number_; ++i)
    prob_mat.col(i) = GMMPredictForGP(i, raw_mean_predictions.col(i));
  // Normalize probability model
  for (int i = 0; i < test_number; ++i) {
    prob_mat.row(i) = prob_mat.row(i) / prob_mat.row(i).array().sum();
  }
  // Calculate weighted preditions
  Eigen::MatrixXd weighted_mean =
      raw_mean_predictions.array() * prob_mat.array();
  Eigen::MatrixXd weighed_var = raw_var_predictions.array() * prob_mat.array();
  pred_mean = weighted_mean.rowwise().sum();
  pred_var = weighed_var.rowwise().sum();
}
}  // namespace gpmm
}  // namespace sampling