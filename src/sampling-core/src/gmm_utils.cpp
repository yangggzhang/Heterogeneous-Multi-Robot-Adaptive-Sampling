#include "sampling-core/gmm_utils.h"
#include <ros/ros.h>

namespace sampling {

Eigen::MatrixXd repmat(const Eigen::VectorXd &X, const int &n) {
  /*
   *Input: X: m x 1
   *Output: Y: m x n, all columns are the same to X
   */
  Eigen::MatrixXd Y(X.size(), n);
  for (int i = 0; i < n; i++) {
    Y.col(i) = X;
  }
  return Y;
}

Eigen::MatrixXd repmat(const Eigen::MatrixXd &x, const int &n) {
  Eigen::VectorXd vector_data = x.col(0);
  return repmat(vector_data, n);
}

Eigen::MatrixXd loggausspdf(const Eigen::MatrixXd &data,
                            const Eigen::MatrixXd &mu,
                            const Eigen::MatrixXd &Sigma) {
  Eigen::MatrixXd unbiased_data = data;
  for (int i = 0; i < data.rows(); i++) {
    unbiased_data.row(i) = data.row(i).array() - mu(i, 0);
  }

  Eigen::MatrixXd U = Sigma.llt().matrixL();
  Eigen::MatrixXd Q = U.inverse() * data;
  Eigen::MatrixXd q = Q.array() * Q.array();
  double c = (double)data.rows() * log(2 * M_PI) +
             2 * U.diagonal().array().log().sum();
  Eigen::MatrixXd y = -1 * (c + q.array()) / 2;
  return y.transpose();
}

void expectation(const Eigen::MatrixXd &data, Model &gp_model, double &exp) {
  int d = data.rows();
  int n = data.cols();
  int k = gp_model.mu.cols();

  for (int i = 0; i < k; i++) {
    gp_model.R.col(i) =
        loggausspdf(data, gp_model.mu.col(i), gp_model.Sigma.block<1, 1>(0, i));
  }

  for (int j = 0; j < k; j++) {
    gp_model.R.col(j) =
        gp_model.R.col(j).array() + gp_model.w.array().log()(0, j);
  }

  Eigen::MatrixXd T =
      gp_model.R.array().exp().rowwise().sum().log(); // T: n x 1

  exp = T.sum() / (double)n;
  for (int i = 0; i < n; i++) {
    gp_model.R.row(i) = gp_model.R.row(i).array() - T(i, 0);
  }
  gp_model.R = gp_model.R.array().exp();
}

void maximization(const Eigen::MatrixXd data, Model &gp_model) {
  double n = (double)data.cols();
  Eigen::MatrixXd nk = gp_model.R.colwise().sum();
  gp_model.Sigma = Eigen::MatrixXd::Zero(1, gp_model.numGaussian);
  gp_model.w = nk / n;
  gp_model.mu = data * gp_model.R;
  gp_model.mu = gp_model.mu.array() / nk.array();
  Eigen::MatrixXd r = gp_model.R.array().sqrt();
  for (int i = 0; i < gp_model.numGaussian; i++) {
    Eigen::MatrixXd Xo_mat = data.array() - gp_model.mu(i);
    Eigen::VectorXd Xo(Eigen::Map<Eigen::VectorXd>(
        Xo_mat.data(), Xo_mat.cols() * Xo_mat.rows()));
    Xo = Xo.array() * r.col(i).array();
    gp_model.Sigma(i) = Xo.dot(Xo) / nk(i);
  }
}

void expectation_maximization(const Eigen::MatrixXd &data,
                              const int &max_iteration, const double &tolerance,
                              Model &gp_model) {
  double last_llh, current_llh;
  Eigen::MatrixXd transpose_data = data.transpose();
  for (int iter = 0; iter < max_iteration; iter++) {
    maximization(transpose_data, gp_model);
    expectation(transpose_data, gp_model, current_llh);
    if (iter == 0) {
      last_llh = current_llh;
      continue;
    } else {
      if (fabs(last_llh - current_llh) < tolerance) {
        double diff = abs(last_llh - current_llh);
        break;
      } else {
        last_llh = current_llh;
      }
    }
  }
}

void GaussianMixture_prediction(const Model &gp_model, Eigen::VectorXi &label) {
  Eigen::MatrixXd::Index max_index;
  label.resize(gp_model.R.rows());
  for (int i = 0; i < gp_model.R.rows(); i++) {
    gp_model.R.row(i).maxCoeff(&max_index);
    label(i) = max_index; // Eigen::VectorXd label(X.cols());
  }
}

void gp_compute(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y,
                const Eigen::MatrixXd &Xtest, Eigen::VectorXd &mu,
                Eigen::VectorXd &s2) {
  double y;
  libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
  Eigen::VectorXd params(3);
  params << 0.5, 0.5, -2.0;
  gp.covf().set_loghyper(params);

  for (int i = 0; i < Y.size(); i++) {
    double x[] = {X(i, 0), X(i, 1)};
    y = Y(i);
    gp.add_pattern(x, y);
  }

  for (int i = 0; i < Xtest.rows(); i++) {
    double x[] = {Xtest(i, 0), Xtest(i, 1)};
    mu(i) = gp.f(x);
    s2(i) = gp.var(x);
  }
}

/*
* Gaussian Process for Machine Learning
* Input:
*       Xs   -   N x d  coordinates for all grids
*       Fs   -   N x 1  Realization of all grids (such as temperature readings)
     ind_train - N_train x 1  index of training data
     Xtest_new  - N_test x d  coordinates for all testing grids
     Ftest_new  - N_test x 1  Realization of all testing grids (ground truth)

output:
    mu - N_test x 1   Predicted value on all testing grids
    s2 - N_test x 1   Variance of prediction on all testing grids
    //rms - 1 x 1     RMS error from ground truth
*/
void gpml_rms(const Eigen::MatrixXd &Xs_train, const Eigen::MatrixXd &Fs_train,
              const Eigen::MatrixXd &X_test, Eigen::VectorXd &mu,
              Eigen::VectorXd &s2) {
  Eigen::MatrixXd Fs_train_mtz;
  Fs_train_mtz = Fs_train.array() - Fs_train.mean(); // mean value equals to
  gp_compute(Xs_train, Fs_train_mtz, X_test, mu, s2);
  mu = mu.array() + Fs_train.mean();
}

bool prepare_MixtureGaussianProcessd_data(
    const Model &gp_model, const Eigen::VectorXi &label,
    const Eigen::MatrixXd &location, const Eigen::MatrixXd &data,
    std::vector<Eigen::MatrixXd> &training_location,
    std::vector<Eigen::MatrixXd> &training_data,
    std::vector<Eigen::MatrixXd> &probability) {

  if (label.size() != location.rows() || label.size() != data.rows()) {
    ROS_INFO_STREAM("Prediction size is not consistant with data size!");
    return false;
  }

  std::vector<std::vector<int>> index;
  index.resize(gp_model.numGaussian);
  for (size_t i = 0; i < label.size(); i++) {
    index[label[i]].push_back(i);
  }

  training_location.clear();
  training_data.clear();
  probability.clear();

  training_location.resize(gp_model.numGaussian);
  training_data.resize(gp_model.numGaussian);
  probability.resize(gp_model.numGaussian);

  for (int i = 0; i < gp_model.numGaussian; i++) {
    training_location[i].resize(index[i].size(), location.cols());
    training_data[i].resize(index[i].size(), data.cols());
    probability[i].resize(index[i].size(), gp_model.R.cols());
    for (int j = 0; j < index[i].size(); j++) {
      training_location[i].row(j) = location.row(index[i][j]);
      training_data[i].row(j) = data.row(index[i][j]);
      probability[i].row(j) = gp_model.R.row(index[i][j]);
    }
  }
  return true;
}

/*
* function for learning the gating function

  input:
         all training data Xs: n x d (10 x 2)
         all labels of probability R   n x K (10 x 3)
         all models model struct with K dimensions

  output:
      predicted probability of gating function for each cluster: n_test x K
      PP_exp:   softmax  (sum is 1): n_test x K
      PP_out:   standard normalize with + and - (sum is 1)
      PP:   raw data of gp predicted probability (sum maybe close to 1)
*/
void GaussianProcess_fix(const Model &gp_model,
                         const std::vector<Eigen::MatrixXd> &training_location,
                         const std::vector<Eigen::MatrixXd> &training_data,
                         const Eigen::MatrixXd &test_data, Eigen::MatrixXd &mu,
                         Eigen::MatrixXd &s2) {

  mu = Eigen::MatrixXd::Zero(test_data.rows(),
                             gp_model.numGaussian); // Sample_size x 3
  s2 = Eigen::MatrixXd::Zero(test_data.rows(),
                             gp_model.numGaussian); // Sample_size x 3

  for (int k = 0; k < gp_model.numGaussian; k++) {
    if (training_location[k].rows() == 0) {
      continue;
    }
    Eigen::VectorXd mu_vec(test_data.rows());
    Eigen::VectorXd s2_vec(test_data.rows());
    gpml_rms(training_location[k], training_data[k], test_data, mu_vec, s2_vec);
    mu.col(k) = mu_vec;
    s2.col(k) = s2_vec;
  }
}

Eigen::MatrixXd validate_matrix(const double &num,
                                const Eigen::MatrixXd &matrix) {
  Eigen::MatrixXd nonnan_matrix = matrix;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      if (std::isnan(matrix(i, j))) {
        nonnan_matrix(i, j) = num;
      }
    }
  }
  return nonnan_matrix;
}

/// mask for nan
Eigen::MatrixXd boolen_mask(const Eigen::MatrixXd &matrix) {
  Eigen::MatrixXd mask = matrix;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      mask(i, j) = 1.0 - (double)std::isnan(matrix(i, j));
    }
  }
  return mask;
}

void normalize_matrix(const bool &row, Eigen::MatrixXd &matrix) {
  if (row) {
    for (int i = 0; i < matrix.rows(); i++) {
      matrix.row(i) = matrix.row(i).array() / matrix.row(i).sum();
    }
  } else {
    for (int j = 0; j < matrix.cols(); j++) {
      matrix.col(j) = matrix.col(j).array() / matrix.col(j).sum();
    }
  }
}

/*
* function for learning the gating function

input:
       all training data Xs: n x d (10 x 2)
       all labels of probability R   n x K (10 x 3)
       all models model struct with K dimensions

output:
    predicted probability of gating function for each cluster: n_test x K
    PP_exp:   softmax  (sum is 1): n_test x K
    PP_out:   standard normalize with + and - (sum is 1)
    PP:   raw data of gp predicted probability (sum maybe close to 1)
*/
Eigen::MatrixXd GaussianProcess_predict(const Model &gp_model,
                                        const Eigen::MatrixXd &data,
                                        const Eigen::MatrixXd &test_data) {

  int n_test = test_data.rows();

  Eigen::MatrixXd prediction_probability =
      Eigen::MatrixXd::Zero(n_test, gp_model.numGaussian); // 202 x 3

  for (int k = 0; k < gp_model.numGaussian; k++) {
    Eigen::VectorXd mu_gp(n_test);
    Eigen::VectorXd s2_gp(n_test);
    gpml_rms(data, gp_model.R.col(k), test_data, mu_gp, s2_gp);
    prediction_probability.col(k) = mu_gp;
  }

  for (int i = 0; i < prediction_probability.rows(); i++) {
    prediction_probability.row(i) = prediction_probability.row(i).array() /
                                    prediction_probability.row(i).sum();
  }

  return prediction_probability;
}

void apply_GP(const Model &gp_model, const Eigen::MatrixXd &mu,
              const Eigen::MatrixXd &s2, const Eigen::MatrixXd &probability,
              Eigen::VectorXd &mean, Eigen::VectorXd &variance) {
  Eigen::MatrixXd mu_mask = boolen_mask(mu);

  /// Mean
  Eigen::MatrixXd normalized_prediction = probability.array() * mu_mask.array();
  normalize_matrix(true, normalized_prediction);
  Eigen::MatrixXd nonnan_mu = validate_matrix(0.0, mu);
  mean = (normalized_prediction.array() * nonnan_mu.array()).rowwise().sum();

  /// Variance
  Eigen::MatrixXd mu_mat = repmat(mean, gp_model.numGaussian);
  Eigen::MatrixXd pred_s2_mat =
      (nonnan_mu - mu_mat).array() * (nonnan_mu - mu_mat).array();
  pred_s2_mat += s2;
  variance = (probability.array() * pred_s2_mat.array()).rowwise().sum();
}

void GaussianProcessMixture_predict(const Eigen::MatrixXd &Xss,
                                    const Eigen::MatrixXd &Fss,
                                    const Eigen::MatrixXd &All_Xss,
                                    Model &gpModel, Eigen::VectorXd &pred_h,
                                    Eigen::VectorXd &pred_Var) {
  Eigen::VectorXi label;
  GaussianMixture_prediction(gpModel, label);

  std::vector<Eigen::MatrixXd> training_location, training_data, probability;
  if (!prepare_MixtureGaussianProcessd_data(gpModel, label, Xss, Fss,
                                            training_location, training_data,
                                            probability)) {
    ROS_INFO_STREAM("Can not prepare training data");
  }

  Eigen::MatrixXd mu, s2;

  GaussianProcess_fix(gpModel, training_location, training_data, All_Xss, mu,
                      s2);

  Eigen::MatrixXd gp_probability =
      GaussianProcess_predict(gpModel, Xss, All_Xss);

  apply_GP(gpModel, mu, s2, gp_probability, pred_h, pred_Var);
}
} // namespace sampling