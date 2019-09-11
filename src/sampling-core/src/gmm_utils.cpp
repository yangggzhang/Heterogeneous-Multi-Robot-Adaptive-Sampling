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

void loggausspdf(const Eigen::MatrixXd data, const Eigen::MatrixXd mu,
                 const Eigen::MatrixXd sigma, Eigen::MatrixXd log_likelyhood) {
  Eigen::MatrixXd mu_matrix = repmat(mu, data.cols());
  Eigen::MatrixXd unbiased_data = data - mu_matrix;
  Eigen::MatrixXd U = sigma.llt().matrixL();
  Eigen::MatrixXd Q = U.inverse() * unbiased_data;
  Eigen::MatrixXd q = Q.array() * Q.array();
  double d = (double)data.rows();

  double c = d * log(2 * M_PI) + 2 * U.diagonal().array().log().sum();
  log_likelyhood = -1 * (c + q.array()) / 2.0;
  log_likelyhood.transposeInPlace();
}

void expectation(const Eigen::MatrixXd &data, Model &gp_model, double &exp) {
  int d = data.rows();
  int n = data.cols();
  int k = gp_model.mu.cols();

  for (int i = 0; i < k; i++) {
    loggausspdf(data, gp_model.mu.col(i), gp_model.Sigma.block<1, 1>(0, i),
                gp_model.R.col(i));
  }

  for (int j = 0; j < k; j++) {
    gp_model.R.col(j) =
        gp_model.R.col(j).array() + gp_model.w.array().log()(0, j);
  }

  Eigen::MatrixXd T =
      gp_model.R.array().exp().rowwise().sum().log();  // T: n x 1

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
    maximization(data, gp_model);
    expectation(data, gp_model, current_llh);
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
  ROS_INFO_STREAM("R size : " << gp_model.R.rows() << " " << gp_model.R.cols());
  ROS_INFO_STREAM("R " << gp_model.R);
  for (int i = 0; i < gp_model.R.rows(); i++) {
    gp_model.R.row(i).maxCoeff(&max_index);
    label(i) = max_index;  // Eigen::VectorXd label(X.cols());
  }
}

void gp_compute(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y,
                const Eigen::MatrixXd &X_test, Eigen::MatrixXd &mu,
                Eigen::MatrixXd &s2) {
  libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
  Eigen::VectorXd params(3);
  params << 0.5, 0.5, -2.0;
  gp.covf().set_loghyper(params);

  double y;
  for (int i = 0; i < Y.size(); i++) {
    double x[] = {X(i, 0), X(i, 1)};
    y = Y(i);
    gp.add_pattern(x, y);
  }

  for (int i = 0; i < X_test.rows(); i++) {
    double x[] = {X_test(i, 0), X_test(i, 1)};
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
              const Eigen::MatrixXd &X_test, Eigen::MatrixXd &mu,
              Eigen::MatrixXd &s2) {
  Eigen::MatrixXd Fs_train_mtz;
  Fs_train_mtz = Fs_train.array() - Fs_train.mean();  // mean value equals to 0
  gp_compute(Xs_train, Fs_train_mtz, X_test, mu, s2);
  mu = mu.array() + Fs_train.mean();
}

bool prepare_MixtureGaussianProcessd_data(
    const Model &gp_model, const Eigen::VectorXi &label,
    const Eigen::MatrixXd &location, const Eigen::MatrixXd &data,
    std::vector<Eigen::MatrixXd> &training_location,
    std::vector<Eigen::MatrixXd> &training_data,
    std::vector<Eigen::MatrixXd> &probability) {
  ROS_INFO_STREAM("P - 0");

  if (label.size() != location.rows() || label.size() != data.rows()) {
    ROS_INFO_STREAM("Prediction size is not consistant with data size!");
    return false;
  }

  std::vector<std::vector<int>> index;
  index.resize(gp_model.numGaussian);
  for (size_t i = 0; i < label.size(); i++) {
    // ROS_INFO_STREAM("label " << label[i]);
    index[label[i]].push_back(i);
  }
  ROS_INFO_STREAM("P - 1");

  training_location.clear();
  training_data.clear();
  probability.clear();

  training_location.resize(gp_model.numGaussian);
  training_data.resize(gp_model.numGaussian);
  probability.resize(gp_model.numGaussian);
  ROS_INFO_STREAM("P - 2");

  for (int i = 0; i < gp_model.numGaussian; i++) {
    ROS_INFO_STREAM("P - 3");

    training_location[i].resize(index[i].size(), data.cols());
    training_data[i].resize(index[i].size(), data.cols());
    probability[i].resize(index[i].size(), data.cols());
    ROS_INFO_STREAM("P - 4");
    for (int j = 0; j < index[i].size(); j++) {
      ROS_INFO_STREAM("size : " << index[i].size());
      training_location[i].row(j) = location.row(index[i][j]);
      training_data[i].row(j) = data.row(index[i][j]);
      probability[i].row(j) = gp_model.R.row(index[i][j]);
    }
    ROS_INFO_STREAM("P - 5");
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
void GaussianProcess_prediction(
    const std::vector<Eigen::MatrixXd> &training_location,
    const std::vector<Eigen::MatrixXd> &training_data,
    const Eigen::MatrixXd &test_data, Eigen::MatrixXd &prediction) {
  int numGaussian = training_location.size();  // get number of models, 3
  int numTest = test_data.rows();

  Eigen::MatrixXd naive_prediction =
      Eigen::MatrixXd::Zero(numTest, numGaussian);  // 202 x 3

  for (int i = 0; i < numGaussian; i++) {
    Eigen::MatrixXd var(numTest, 1);
    Eigen::MatrixXd mu(numTest, 1);
    gpml_rms(training_location[i], training_data[i], test_data, mu, var);
    naive_prediction.col(i) = mu;
  }

  prediction.resize(numTest, numGaussian);

  for (int i = 0; i < prediction.rows(); i++) {
    prediction.row(i) =
        naive_prediction.row(i).array() / naive_prediction.row(i).sum();
  }
}

void fill_nan(const double &num, Eigen::MatrixXd &matrix) {
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      if (std::isnan(matrix(i, j))) {
        matrix(i, j) = num;
      }
    }
  }
}

/// mask for nan
void boolen_mask(const Eigen::MatrixXd &matrix, Eigen::MatrixXd &mask) {
  mask = matrix;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      mask(i, j) = (double)std::isnan(matrix(i, j));
    }
  }
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

bool MixtureGaussianProcess_prediction(const Model &gp_model,
                                       const Eigen::MatrixXd &location,
                                       const Eigen::MatrixXd &data,
                                       const Eigen::MatrixXd &location_test,
                                       Eigen::VectorXd &pred_mu,
                                       Eigen::VectorXd &pred_var) {
  Eigen::VectorXi prediction_label;
  ROS_INFO_STREAM("0");
  GaussianMixture_prediction(gp_model, prediction_label);
  ROS_INFO_STREAM("1");
  std::vector<Eigen::MatrixXd> training_location, training_data, probability;
  if (!prepare_MixtureGaussianProcessd_data(gp_model, prediction_label,
                                            location, data, training_location,
                                            training_data, probability)) {
    ROS_INFO_STREAM("Can not prepare training data");
  }
  ROS_INFO_STREAM("2");
  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(
      location_test.rows(), gp_model.numGaussian);  // Sample_size x 3
  Eigen::MatrixXd s2 = Eigen::MatrixXd::Zero(
      location_test.rows(), gp_model.numGaussian);  // Sample_size x 3
  ROS_INFO_STREAM("3");

  for (int i = 0; i < gp_model.numGaussian; i++) {
    Eigen::MatrixXd local_mu(location_test.rows(), 1);
    Eigen::MatrixXd local_s2(location_test.rows(), 1);
    gpml_rms(training_location[i], training_data[i], location_test, local_mu,
             local_s2);
    mu.col(i) = local_mu;
    s2.col(i) = local_s2;
  }
  ROS_INFO_STREAM("4");

  Eigen::MatrixXd mu_mask;
  boolen_mask(mu, mu_mask);
  ROS_INFO_STREAM("5");

  Eigen::MatrixXd gp_prediction;
  GaussianProcess_prediction(training_location, probability, location_test,
                             gp_prediction);
  ROS_INFO_STREAM("6");

  /// calculate mean value
  Eigen::MatrixXd normalized_prediction =
      gp_prediction.array() * mu_mask.array();
  normalize_matrix(true, normalized_prediction);
  fill_nan(0.0, mu);
  pred_mu = (normalized_prediction.array() * mu.array()).rowwise().sum();
  ROS_INFO_STREAM("7");

  /// calculate
  Eigen::MatrixXd mu_mat = repmat(pred_mu, gp_model.numGaussian);
  Eigen::MatrixXd pred_s2_mat = (mu - mu_mat).array() * (mu - mu_mat).array();
  pred_s2_mat += s2;
  pred_var = (gp_prediction.array() * pred_s2_mat.array()).rowwise().sum();
  ROS_INFO_STREAM("8");
}
}  // namespace sampling