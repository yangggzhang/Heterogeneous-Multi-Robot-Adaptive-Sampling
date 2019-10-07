/**
 * Utility functions for GP update
 * AUTHOR: Yang Zhang
 */

#pragma once

#include <stdlib.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <string>
#include <vector>
#include "gp.h"
#include "gp_utils.h"

namespace sampling {
namespace gmm {

struct Model {
  int numGaussian;
  Eigen::MatrixXd mu;
  Eigen::MatrixXd Sigma;
  Eigen::MatrixXd w;
  Eigen::MatrixXd R;
};

class Gaussian_Mixture_Model {
 public:
  Gaussian_Mixture_Model(){};

  Gaussian_Mixture_Model(const int &num_gaussian,
                         const std::vector<double> &gp_hyperparameter,
                         const Eigen::MatrixXd &training_location,
                         const Eigen::MatrixXd &training_feature,
                         const Eigen::MatrixXd &testing_location);

  void expectation_maximization(const int &max_iteration,
                                const double &tolerance);

  void GaussianProcessMixture_predict(Eigen::VectorXd &pred_h,
                                      Eigen::VectorXd &pred_Var);

  void add_training_data(const Eigen::MatrixXd &new_training_location,
                         const Eigen::MatrixXd &new_training_feature);

 private:
  Eigen::MatrixXd repmat(const Eigen::VectorXd &X, const int &n);

  Eigen::MatrixXd repmat(const Eigen::MatrixXd &x, const int &n);

  Eigen::MatrixXd loggausspdf(const Eigen::MatrixXd &data,
                              const Eigen::MatrixXd &mu,
                              const Eigen::MatrixXd &Sigma);

  void expectation(const Eigen::MatrixXd &data, double &exp);

  void maximization(const Eigen::MatrixXd data);

  void gp_compute(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y,
                  const Eigen::MatrixXd &Xtest, Eigen::VectorXd &mu,
                  Eigen::VectorXd &s2);

  void gpml_rms(const Eigen::MatrixXd &Xs_train,
                const Eigen::MatrixXd &Fs_train, const Eigen::MatrixXd &X_test,
                Eigen::VectorXd &mu, Eigen::VectorXd &s2);

  bool prepare_MixtureGaussianProcessd_data(
      const Eigen::VectorXi &label, const Eigen::MatrixXd &location,
      const Eigen::MatrixXd &data,
      std::vector<Eigen::MatrixXd> &training_location,
      std::vector<Eigen::MatrixXd> &training_data,
      std::vector<Eigen::MatrixXd> &probability);

  void GaussianProcess_fix(Eigen::MatrixXd &mu, Eigen::MatrixXd &s2);

  void GaussianMixture_prediction(Eigen::VectorXi &label);

  Eigen::MatrixXd boolen_mask(const Eigen::MatrixXd &matrix);

  void normalize_matrix(const bool &row, Eigen::MatrixXd &matrix);

  Eigen::MatrixXd validate_matrix(const double &num,
                                  const Eigen::MatrixXd &matrix);

  void apply_GP(const Eigen::MatrixXd &mu, const Eigen::MatrixXd &s2,
                const Eigen::MatrixXd &probability, Eigen::VectorXd &mean,
                Eigen::VectorXd &variance);

  Eigen::MatrixXd GaussianProcess_predict(const Eigen::MatrixXd &data,
                                          const Eigen::MatrixXd &test_data);
  libgp::GaussianProcess *gp_model_;
  Model model_;
  Eigen::MatrixXd training_location_;
  Eigen::MatrixXd training_feature_;
  Eigen::MatrixXd transpose_training_location_;
  Eigen::MatrixXd testing_location_;
  std::vector<Eigen::MatrixXd> training_location_vec_;
  std::vector<Eigen::MatrixXd> training_feature_vec_;
  std::vector<Eigen::MatrixXd> probability_vec_;
  Eigen::VectorXi label_;
};
}  // namespace gmm
}  // namespace sampling