#include "gp_utils.h"

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

bool loggausspdf(const Eigen::MatrixXd data, const Eigen::MatrixXd mu,
                 const Eigen::MatrixXd sigma, Eigen::MatrixXd log_likelyhood) {
  if (data.rows() != mu.rows()) {
    return false;
  }

  Eigen::MatrixXd mu_matrix = mu.colwise.replicate(1, data.cols());
  Eigen::MatrixXd unbiased_data = data - mu_matrix;
  Eigen::MatrixXd U = Sigma.llt().matrixL();
  Eigen::MatrixXd Q = U.inverse() * unbiased_data;
  Eigen::MatrixXd q = Q.array() * Q.array();

  double c = d * log(2 * M_PI) + 2 * U.diagonal().array().log().sum();

  Eigen::MatrixXd log_likelyhood = -1 * (c + q.array()) / 2;
  log_likelyhood = log_likelyhood.transpose();
  return true;
}

void expectation(const Eigen::MatrixXd &data, const Model &gp_model,
                 double &exp) {
  int d = data.rows();
  int n = data.cols();
  int k = gp_model.mu.cols();

  for (int i = 0; i < k; i++) {
    if (!loggausspdf(data, gp_model.mu.col(i), gp_model.Sigma.block<1, 1>(0, i),
                     gp_model.R.col(i))) {
      return false;
    }
  }

  for (int j = 0; j < k; j++) {
    gp_model.R.col(j).array() += gp_model.w.array().log()(0, j);
  }

  Eigen::MatrixXd T =
      gp_model.R.array().exp().rowwise().sum().log(); // T: n x 1
  exp = T.sum() / (double)n;
  for (int i = 0; i < n; i++) {
    gp_model.R.row(i).array() -= T(i, 0);
  }
  gp_model.R = gp_model.R.array().exp();
}

void maximization(const Eigen::MatrixXd data, Model &gp_model) {
  double n = (double)data.cols();
  Eigen::MatrixXd nk = gp_model.R.colwise().sum();
  model.w = nk / n;
  model.mu = data * gp_model.R / nk.array();
  Eigen::MatrixXd r = R.array().sqrt();
  Eigen::VectorXd x_vector(
      Eigen::Map<Eigen::VectorXd>(X.data(), X.cols() * X.rows()));
  for (int i = 0; i < gp_model.numGaussian; i++) {
    Eigen::VectorXd Xo = x_vector.array() - mu(i);
    ;
    Xo = Xo.array() * r.col(i).array();
    model.Sigma(i) = Xo.dot(Xo) / nk(i);
  }
}

void expectation_maximization(const Eigen::MatrixXd &data,
                              const int &max_iteration, const double &tolerance,
                              Model &gp_model) {
  double last_llh, current_llh;
  data.transposeInPlace();
  for (int iter = 0; iter < max_iteration; iter++) {
    maximization(data, gp_model);
    expectation(data, gp_model, current_llh);
    if (iter == 0) {
      last_llh = current_llh;
      continue;
    } else {
      if (abs(last_llh - current_llh) < tolerance) {
        break;
      } else {
        last_llh = current_llh;
      }
    }
  }
}

void GaussianMixture_prediction(const Model &gp_model, Eigen::VectorXi &label) {
  Eigen::MatrixXd::Index max_index;
  for (int i = 0; i < gp_model.R.rows(); i++) {
    gp_model.R.row(i).maxCoeff(&max_index);
    label(i) = max_index; // Eigen::VectorXd label(X.cols());
  }
}

void gp_compute(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y,
                const Eigen::MatrixXd &X_test, Eigen::MatrixXd &mu,
                Eigen::MatrixXd &s2) {
  GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
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
  Fs_train_mtz = Fs_train.array() - Fs_train.mean(); // mean value equals to 0
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
    training_location[i].resize(index[i].size(), data.cols());
    training_data[i].resize(index[i].size(), data.cols());
    probability[i].resize(index[i].size(), data.cols());
    for (int j = 0; j < index[i].size(); j++) {
      training_location[i].row(j) = location.row(index[i][j]);
      training_data[i].row(j) = data.row(index[i][j]);
      probability[i].row(j) = gp_model.R(index[i][j], i);
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
void GaussianProcess_prediction(
    const std::vector<Eigen::MatrixXd> &training_location,
    const std::vector<Eigen::MatrixXd> &training_data,
    const Eigen::MatrixXd &test_data, Eigen::MatrixXd &prediction) {
  int numGaussian = training_location.size(); // get number of models, 3
  int N = training_data.rows();               // 10
  int numTest = test_data.rows();

  Eigen::MatrixXd naive_prediction =
      Eigen::MatrixXd::Zero(numTest, numGaussian); // 202 x 3

  for (int i = 0; i < numGaussian; i++) {
    Eigen::MatrixXd var(numTest, 1);
    gpml_rms(training_location[i], training_data[i], test_data,
             naive_prediction.col(i), var);
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
      if (isnan(matrix(i, j))) {
        matrix(i, j) = num;
      }
    }
  }
}

/// mask for nan
void boolen_mask(const Eigen::MatrixXd &matrix, Eigen::MatrixX &mask) {
  mask = matrix;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      mask(i, j) = (double)isnan(matrix(i, j));
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
  GaussianMixture_prediction(gp_model, prediction_label);
  std::vector<Eigen::MatrixXd> training_location, training_data, probability;
  if (!prepare_MixtureGaussianProcessd_data(gp_model, prediction_label,
                                            location, data, training_location,
                                            training_data, probability)) {
    ROS_INFO_STREAM("Can not prepare training data");
  }

  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(
      location_test.rows(), gpModel.numGaussian); // Sample_size x 3
  Eigen::MatrixXd s2 = Eigen::MatrixXd::Zero(
      location_test.rows(), gpModel.numGaussian); // Sample_size x 3

  for (int i = 0; i < gpModel.numGaussian; i++) {
    gpml_rms(training_location[i], training_data[i], location_test, mu.col(i),
             s2.col(i));
  }

  Eigen::MatrixXd mu_mask;
  boolen_mask(mu, mu_mask);

  Eigen::MatrixXd gp_prediction;
  GaussianProcess_prediction(training_location, probability, location_test,
                             gp_prediction);

  /// calculate mean value
  Eigen::MatrixXd normalized_prediction =
      gp_prediction.array() * boolen_mask.array();
  normalize_matrix(true, normalized_prediction);
  fill_nan(0.0, mu);
  pred_mu = (normalized_prediction.array() * mu.array()).rowwise().sum();

  /// calculate
  Eigen::MatrixXd mu_mat = repmat(pred_mu, gp_model.numGaussian);
  Eigen::MatrixXd pred_s2_mat = (mu - mu_mat).array() * (mu - mu_mat).array();
  pred_s2_mat += s2;
  pred_var = (gp_prediction.array() * pred_s2_mat.array()).rowwise().sum();
}
} // namespace sampling