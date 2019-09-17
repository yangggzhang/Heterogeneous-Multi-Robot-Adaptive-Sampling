/**
 * Utility functions for GP update
 * AUTHOR: Yang Zhang
 */

#pragma once

#include "gp.h"
#include "gp_utils.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <vector>

namespace sampling {

struct Model {
  int numGaussian;
  Eigen::MatrixXd mu;
  Eigen::MatrixXd Sigma;
  Eigen::MatrixXd w;
  Eigen::MatrixXd R;
};

Eigen::MatrixXd repmat(const Eigen::VectorXd &X, const int &n);

Eigen::MatrixXd repmat(const Eigen::MatrixXd &x, const int &n);

Eigen::MatrixXd loggausspdf(const Eigen::MatrixXd &data,
                            const Eigen::MatrixXd &mu,
                            const Eigen::MatrixXd &Sigma);

void expectation(const Eigen::MatrixXd &data, Model &gp_model, double &exp);

void maximization(const Eigen::MatrixXd data, Model &gp_model);

void expectation_maximization(const Eigen::MatrixXd &data,
                              const int &max_iteration, const double &tolerance,
                              Model &gp_model);

void gp_compute(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y,
                const Eigen::MatrixXd &X_test, Eigen::MatrixXd &mu,
                Eigen::MatrixXd &s2);

void gpml_rms(const Eigen::MatrixXd &Xs_train, const Eigen::MatrixXd &Fs_train,
              const Eigen::MatrixXd &X_test, Eigen::VectorXd &mu,
              Eigen::VectorXd &s2);

void GaussianMixture_prediction(const Model &gp_model, Eigen::VectorXi &label);

bool prepare_MixtureGaussianProcessd_data(
    const Model &gp_model, const Eigen::VectorXi &label,
    const Eigen::MatrixXd &location, const Eigen::MatrixXd &data,
    std::vector<Eigen::MatrixXd> &training_location,
    std::vector<Eigen::MatrixXd> &training_data,
    std::vector<Eigen::MatrixXd> &probability);

void GaussianProcess_fix(const Model &gp_model,
                         const std::vector<Eigen::MatrixXd> &training_location,
                         const std::vector<Eigen::MatrixXd> &training_data,
                         const Eigen::MatrixXd &test_data, Eigen::MatrixXd &mu,
                         Eigen::MatrixXd &s2);

void boolen_mask(const Eigen::MatrixXd &matrix, Eigen::MatrixXd &mask);

void normalize_matrix(const bool &row, Eigen::MatrixXd &matrix);

Eigen::MatrixXd validate_matrix(const double &num,
                                const Eigen::MatrixXd &matrix);

void apply_GP(const Model &gp_model, const Eigen::MatrixXd &mu,
              const Eigen::MatrixXd &s2, const Eigen::MatrixXd &probability,
              Eigen::VectorXd &mean, Eigen::VectorXd &variance);

Eigen::MatrixXd GaussianProcess_predict(const Model &gp_model,
                                        const Eigen::MatrixXd &data,
                                        const Eigen::MatrixXd &test_data);

void GaussianProcessMixture_predict(const Eigen::MatrixXd &Xss,
                                    const Eigen::MatrixXd &Fss,
                                    const Eigen::MatrixXd &All_Xss,
                                    Model &gpModel, Eigen::VectorXd &pred_h,
                                    Eigen::VectorXd &pred_Var);
} // namespace sampling