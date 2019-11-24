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
                         const std::vector<double> &gp_hyperparameter);

  void ExpectationAndMaximization(const int &max_iteration,
                                  const double &tolerance);

  void GaussianProcessMixturePredict(const Eigen::MatrixXd &All_Xss,
                                     Eigen::VectorXd &pred_h,
                                     Eigen::VectorXd &pred_Var);

  void AddTrainingData(const Eigen::MatrixXd &new_training_location,
                       const Eigen::MatrixXd &new_training_feature);

 private:
  Eigen::MatrixXd repmat(const Eigen::VectorXd &X, const int &n);

  Eigen::MatrixXd repmat(const Eigen::MatrixXd &x, const int &n);

  Eigen::MatrixXd loggausspdf(const Eigen::MatrixXd &data,
                              const Eigen::MatrixXd &mu,
                              const Eigen::MatrixXd &Sigma);

  void Expectation(const Eigen::MatrixXd &data, Model &gp_model,
                   double &prob_change);

  void Maximization(const Eigen::MatrixXd &data, Model &gp_model);

  void GPCompute(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y,
                 const Eigen::MatrixXd &Xtest, Eigen::VectorXd &mu,
                 Eigen::VectorXd &s2);

  void GPML_RMS(const Eigen::MatrixXd &Xs_train,
                const Eigen::MatrixXd &Fs_train, const Eigen::MatrixXd &X_test,
                Eigen::VectorXd &mu, Eigen::VectorXd &s2);

  void GaussianMixturePrediction(const Model &gp_model, Eigen::VectorXi &label);

  bool PrepareMixtureGaussianProcessdData(
      const Model &gp_model, const Eigen::VectorXi &label,
      const Eigen::MatrixXd &location, const Eigen::MatrixXd &data,
      std::vector<Eigen::MatrixXd> &training_location,
      std::vector<Eigen::MatrixXd> &training_data,
      std::vector<Eigen::MatrixXd> &probability);

  void GaussianProcessFix(const Model &gp_model,
                          const std::vector<Eigen::MatrixXd> &training_location,
                          const std::vector<Eigen::MatrixXd> &training_data,
                          const Eigen::MatrixXd &test_data, Eigen::MatrixXd &mu,
                          Eigen::MatrixXd &s2);

  Eigen::MatrixXd BoolenMask(const Eigen::MatrixXd &matrix);

  void NormalizeMatrix(const bool &row, Eigen::MatrixXd &matrix);

  Eigen::MatrixXd ValidateMatrix(const double &num,
                                 const Eigen::MatrixXd &matrix);

  void ApplyGP(const Model &gp_model, const Eigen::MatrixXd &mu,
               const Eigen::MatrixXd &s2, const Eigen::MatrixXd &probability,
               Eigen::VectorXd &mean, Eigen::VectorXd &variance);

  Eigen::MatrixXd GaussianProcessPredict(const Model &gp_model,
                                         const Eigen::MatrixXd &data,
                                         const Eigen::MatrixXd &test_data);

  Model model_;
  libgp::GaussianProcess *gp_model_;

  Eigen::MatrixXd training_location_;
  Eigen::MatrixXd training_feature_;
  Eigen::MatrixXd transpose_training_feature_;
};
}  // namespace gmm
}  // namespace sampling