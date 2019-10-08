#include "sampling_core/gmm_utils.h"
#include <ros/ros.h>

namespace sampling {
namespace gmm {

Eigen::MatrixXd Gaussian_Mixture_Model::repmat(const Eigen::VectorXd &X, const int &n) {
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

Eigen::MatrixXd Gaussian_Mixture_Model::repmat(const Eigen::MatrixXd &x, const int &n) {
  Eigen::VectorXd vector_data = x.col(0);
  return repmat(vector_data, n);
}

Eigen::MatrixXd Gaussian_Mixture_Model::loggausspdf(const Eigen::MatrixXd &data,
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

void Gaussian_Mixture_Model::expectation(const Eigen::MatrixXd &data, Model &gp_model, double &exp) {
  int d = data.rows();
  int n = data.cols();
  int k = model_.mu.cols();

  for (int i = 0; i < k; i++) {
    model_.R.col(i) =
        loggausspdf(data, model_.mu.col(i), model_.Sigma.block<1, 1>(0, i));
  }

  for (int j = 0; j < k; j++) {
    model_.R.col(j) = model_.R.col(j).array() + model_.w.array().log()(0, j);
  }

  Eigen::MatrixXd T = model_.R.array().exp().rowwise().sum().log();  // T: n x 1

  exp = T.sum() / (double)n;
  for (int i = 0; i < n; i++) {
    model_.R.row(i) = model_.R.row(i).array() - T(i, 0);
  }
  model_.R = model_.R.array().exp();
}

void Gaussian_Mixture_Model::maximization(const Eigen::MatrixXd& data, Model &gp_model) {
  double n = (double)data.cols();
  Eigen::MatrixXd nk = model_.R.colwise().sum();
  model_.Sigma = Eigen::MatrixXd::Zero(1, model_.numGaussian);
  model_.w = nk / n;
  model_.mu = data * model_.R;
  model_.mu = model_.mu.array() / nk.array();
  Eigen::MatrixXd r = model_.R.array().sqrt();
  for (int i = 0; i < model_.numGaussian; i++) {
    Eigen::MatrixXd Xo_mat = data.array() - model_.mu(i);
    Eigen::VectorXd Xo(Eigen::Map<Eigen::VectorXd>(
        Xo_mat.data(), Xo_mat.cols() * Xo_mat.rows()));
    Xo = Xo.array() * r.col(i).array();
    model_.Sigma(i) = Xo.dot(Xo) / nk(i);
  }
}

void Gaussian_Mixture_Model::expectation_maximization( const int &max_iteration, const double &tolerance) {
  double last_llh, current_llh;
  for (int iter = 0; iter < max_iteration; iter++) {
    maximization(transpose_training_feature_, model_);
    expectation(transpose_training_feature_, model_, current_llh);
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

void Gaussian_Mixture_Model::GaussianMixture_prediction(const Model &gp_model, Eigen::VectorXi &label) {
  Eigen::MatrixXd::Index max_index;
  assert(label.size() == model_.R.rows());
  for (int i = 0; i < model_.R.rows(); i++) {
    model_.R.row(i).maxCoeff(&max_index);
    label(i) = max_index;  // Eigen::VectorXd label(X.cols());
  }
}

void Gaussian_Mixture_Model::gp_compute(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y,
                const Eigen::MatrixXd &Xtest, Eigen::VectorXd &mu,
                Eigen::VectorXd &s2) {
  double y;
  gp_model_->clear_sampleset();

  for (int i = 0; i < Y.size(); i++) {
    double x[] = {X(i, 0), X(i, 1)};
    y = Y(i);
    gp_model_->add_pattern(x, y);
  }

  for (int i = 0; i < Xtest.rows(); i++) {
    double x[] = {Xtest(i, 0), Xtest(i, 1)};
    mu(i) = gp_model_->f(x);
    s2(i) = gp_model_->var(x);
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
void Gaussian_Mixture_Model::gpml_rms(const Eigen::MatrixXd &Xs_train, const Eigen::MatrixXd &Fs_train,
              const Eigen::MatrixXd &X_test, Eigen::VectorXd &mu,
              Eigen::VectorXd &s2) {
  Eigen::MatrixXd Fs_train_mtz;
  Fs_train_mtz = Fs_train.array() - Fs_train.mean(); // mean value equals to
  gp_compute(Xs_train, Fs_train_mtz, X_test, mu, s2);
  mu = mu.array() + Fs_train.mean();
}

bool Gaussian_Mixture_Model::prepare_MixtureGaussianProcessd_data(
    std::vector<Eigen::MatrixXd> &probability) {
  if (label.size() != location.rows() || label.size() != data.rows()) {
    return false;
  }

  std::vector<std::vector<int>> index;
  index.resize(model_.numGaussian);
  for (size_t i = 0; i < label.size(); i++) {
    index[label[i]].push_back(i);
  }

  training_location.clear();
  training_data.clear();
  probability.clear();

  training_location.resize(model_.numGaussian);
  training_data.resize(model_.numGaussian);
  probability.resize(model_.numGaussian);

  for (int i = 0; i < model_.numGaussian; i++) {
    training_location[i].resize(index[i].size(), location.cols());
    training_data[i].resize(index[i].size(), data.cols());
    probability[i].resize(index[i].size(), model_.R.cols());
    for (int j = 0; j < index[i].size(); j++) {
      training_location[i].row(j) = location.row(index[i][j]);
      training_data[i].row(j) = data.row(index[i][j]);
      probability[i].row(j) = model_.R.row(index[i][j]);
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
void Gaussian_Mixture_Model::GaussianProcess_fix(const Model &gp_model,
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
    Eigen::VectorXd mu_vec(testing_location_.rows());
    Eigen::VectorXd s2_vec(testing_location_.rows());
    gpml_rms(training_location_vec_[k], training_feature_vec_[k],
             testing_location_, mu_vec, s2_vec);
    mu.col(k) = mu_vec;
    s2.col(k) = s2_vec;
  }
}

Eigen::MatrixXd Gaussian_Mixture_Model::validate_matrix(const double &num,
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
Eigen::MatrixXd Gaussian_Mixture_Model::boolen_mask(const Eigen::MatrixXd &matrix) {
  Eigen::MatrixXd mask = matrix;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      mask(i, j) = 1.0 - (double)std::isnan(matrix(i, j));
    }
  }
  return mask;
}

void Gaussian_Mixture_Model::normalize_matrix(const bool &row, Eigen::MatrixXd &matrix) {
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
Eigen::MatrixXd Gaussian_Mixture_Model::GaussianProcess_predict(const Model &gp_model,
                                        const Eigen::MatrixXd &data,
                                        const Eigen::MatrixXd &test_data) {

  int n_test = test_data.rows();

  Eigen::MatrixXd prediction_probability =
      Eigen::MatrixXd::Zero(n_test, model_.numGaussian);  // 202 x 3

  for (int k = 0; k < model_.numGaussian; k++) {
    Eigen::VectorXd mu_gp(n_test);
    Eigen::VectorXd s2_gp(n_test);
    gpml_rms(data, model_.R.col(k), test_data, mu_gp, s2_gp);
    prediction_probability.col(k) = mu_gp;
  }

  for (int i = 0; i < prediction_probability.rows(); i++) {
    prediction_probability.row(i) = prediction_probability.row(i).array() /
                                    prediction_probability.row(i).sum();
  }

  return prediction_probability;
}

void Gaussian_Mixture_Model::apply_GP(const Model &gp_model, const Eigen::MatrixXd &mu,
              const Eigen::MatrixXd &s2, const Eigen::MatrixXd &probability,
              Eigen::VectorXd &mean, Eigen::VectorXd &variance) {
  Eigen::MatrixXd mu_mask = boolen_mask(mu);

  /// Mean
  Eigen::MatrixXd normalized_prediction = probability.array() * mu_mask.array();
  normalize_matrix(true, normalized_prediction);
  Eigen::MatrixXd nonnan_mu = validate_matrix(0.0, mu);
  mean = (normalized_prediction.array() * nonnan_mu.array()).rowwise().sum();

  /// Variance
  Eigen::MatrixXd mu_mat = repmat(mean, model_.numGaussian);
  Eigen::MatrixXd pred_s2_mat =
      (nonnan_mu - mu_mat).array() * (nonnan_mu - mu_mat).array();
  pred_s2_mat += s2;
  variance = (probability.array() * pred_s2_mat.array()).rowwise().sum();
}

void Gaussian_Mixture_Model::GaussianProcessMixture_predict(
                                    const Eigen::MatrixXd &All_Xss, Eigen::VectorXd &pred_h,
                                    Eigen::VectorXd &pred_Var) {
  Eigen::VectorXi label;
  GaussianMixture_prediction(model_, label);

  std::vector<Eigen::MatrixXd> training_location, training_data, probability;
  if (!prepare_MixtureGaussianProcessd_data(model_, label, training_location_, training_feature_,
                                            training_location, training_data,
                                            probability)) {
    ROS_INFO_STREAM("Can not prepare training data");
  }

  Eigen::MatrixXd mu, s2;

  GaussianProcess_fix(model_, training_location, training_data, All_Xss, mu,
                      s2);

  Eigen::MatrixXd gp_probability =
      GaussianProcess_predict(model_, training_location_, All_Xss);

  apply_GP(model_, mu, s2, gp_probability, pred_h, pred_Var);
}

void Gaussian_Mixture_Model::add_training_data(
    const Eigen::MatrixXd &new_training_location,
    const Eigen::MatrixXd &new_training_feature) {
  const size_t original_datasize = training_location_.rows();
  training_location_.conservativeResize(
      new_training_location.rows() + training_location_.rows(), 2);
  training_feature_.conservativeResize(
      new_training_feature.rows() + training_feature_.rows(), 1);
  model_.R.conservativeResize(training_location_.rows(), model_.numGaussian);
  assert(training_location_.rows() == training_feature_.rows());
  Eigen::MatrixXd random_probability =
      Eigen::MatrixXd::Random(1, model_.numGaussian);
  random_probability = random_probability.array().abs();
  random_probability.row(0) =
      random_probability.row(0).array() / random_probability.row(0).sum();
  for (size_t i = 0; i < new_training_location.rows(); i++) {
    training_location_.row(original_datasize + i) =
        new_training_location.row(i);
    training_feature_.row(original_datasize + i) = new_training_feature.row(i);
    model_.R.row(original_datasize + i) = random_probability.row(0);
  }
  transpose_training_feature_ = training_feature_.transpose();
  // label_.conservativeResize(training_location_.rows());
}

Gaussian_Mixture_Model::Gaussian_Mixture_Model(const int& num_gaussian, const std::vector<double> &gp_hyperparameter)
{
  model_.numGaussian = num_gaussian;
  assert(gp_hyperparameter.size() == 3);
  gp_model_ = new libgp::GaussianProcess(2, "CovSum ( CovSEiso, CovNoise)");
  Eigen::VectorXd params(3);
  params << gp_hyperparameter[0], gp_hyperparameter[1], gp_hyperparameter[2];
  gp_model_->covf().set_loghyper(params);
}

}

}  // namespace gmm
}  // namespace sampling