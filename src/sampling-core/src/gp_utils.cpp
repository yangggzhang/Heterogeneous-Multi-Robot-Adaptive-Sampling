#include "gp_utils.h"
#include <vector>

namespace sampling {

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
      gp_model.R.array().exp().rowwise().sum().log();  // T: n x 1
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
    label(i) = max_index;  // Eigen::VectorXd label(X.cols());
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
  int numGaussian = training_location.size();  // get number of models, 3
  int N = training_data.rows();                // 10
  int numTest = test_data.rows();

  Eigen::MatrixXd naive_prediction =
      Eigen::MatrixXd::Zero(numTest, numGaussian);  // 202 x 3

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

void filter_zero_element(Eigen::MatrixXd &matrix)
{
  for (int i = 0; i < matrix.rows(); i++)
  {
    for (int j = 0; j < matrix.cols(); j++)
    {
      if (isnan(matrix(i,j))){
        matrix(i,j) = 0;
      }
    }
  }
}

void filter_prediction(const Eigen::MatrixXd mu) {
  /*
   *start to filter infeasible component
   */

  Eigen::MatrixXd pred_mu_mat = mu;  // 202 x 3
  Eigen::MatrixXd pred_mu_mat_tmp(mu.rows(), mu.cols());
  for (int i = 0; i < PP_out.rows(); i++) {
    for (int j = 0; j < PP_out.cols(); j++) {
      pred_mu_mat_tmp(i, j) = 1 - pred_mu_mat.array().isNaN()(i, j);
    }
  }

  Eigen::MatrixXd PP_out_tmp = PP_out.array() * pred_mu_mat_tmp.array();
  Eigen::MatrixXd norm_PP_out(PP_out_tmp.rows(), PP_out_tmp.cols());
  for (int i = 0; i < norm_PP_out.rows(); i++) {
    norm_PP_out.row(i) = PP_out_tmp.row(i).array() / PP_out_tmp.row(i).sum();
  }

  pred_mu_mat = set_NaN_tz(pred_mu_mat);  // set NaN elements to 0
                                          /*
                                           * end of filtering
                                           */
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
      location_test.rows(), gpModel.numGaussian);  // Sample_size x 3
  Eigen::MatrixXd s2 = Eigen::MatrixXd::Zero(
      location_test.rows(), gpModel.numGaussian);  // Sample_size x 3

  for (int i = 0; i < gpModel.numGaussian; i++) {
    gpml_rms(training_location[i], training_data[i], location_test, mu.col(i),
             s2.col(i));
  }

  Eigen::MatrixXd gp_prediction;
  GaussianProcess_prediction(training_location, probability, location_test,
                             gp_prediction);

  // Eigen::MatrixXd pred_h = (norm_PP_out.array() *
  // pred_mu_mat.array()).rowwise().sum();//202 x 1
  pred_h =
      (norm_PP_out.array() * pred_mu_mat.array()).rowwise().sum();  // 202 x 1

  Eigen::MatrixXd pred_s2_mat = s2;

  Eigen::MatrixXd muu_pp_rep = repmat(pred_h, gpModel.numGaussian);  // 202 x 3
  pred_s2_mat =
      (pred_mu_mat - muu_pp_rep).array() * (pred_mu_mat - muu_pp_rep).array();
  pred_s2_mat = pred_s2_mat + s2;
  pred_Var = (PP_out.array() * pred_s2_mat.array()).rowwise().sum();
  // cout<<"max: "<<pred_Var.maxCoeff()<<" min:"<<pred_Var.minCoeff()<<endl;

  // priority_queue< pair<double,int>, vector<pair<double,int>>,
  // less<pair<double,int>>> ucb_pq;
  for (int i = 0; i < pred_h.rows(); i++) {
    double beta = sqrt(UCB_CONSTANT / ((double)collectCount[All_Xss.row(i)]));
    this->phi(i) = pred_Var(i, 0);
    // ucb_pq.push(make_pair(this->phi(i), i));
  }
}

bool distance_2d(const Eigen::MatrixXd &location_1,
                 const Eigen::MatrixXd &location_2, Eigen::MatrixXd &distance) {
  if (location_1.columns() != 2 || location_2.columns() != 2) {
    return false;
  }
  distance.resize(location_1.rows(), location_2.rows());
  for (int i = 0; i < location_1.rows(); i++) {
    for (int j = 0; j < location_2.rows(); j++) {
      distance(i, j) = (location_1.row(i) - location_2.row(j)).norm();
    }
  }
  return true;
}

bool closest_index(const Eigen::MatrixXd &robots_location,
                   const Eigen::MatrixXd &cell_location,
                   std::vector<std::vector<size_t>> &closest_cell) {
  closest_cell.clear();
  /// robots_location.rows == number of robots
  closest_cell.resize(robots_location.rows());
  Eigen::MatrixXd distance;
  if (!distance_2d(robots_location, cell_location, distance)) {
    return false;
  }
  Eigen::MatrixXd::Index min_index;
  for (int i = 0; i < distance.columns(); i++) {
    distance.col(i).minCoeff(&min_index);
    closest_cell[(int)min_index].push_back(i);
  }
  return true;
}

Eigen::MatrixXd extract_rows(const Eigen::MatrixXd &X, const vector<int> &ind) {
  Eigen::MatrixXd X_train(ind.size(), X.cols());

  for (int i = 0; i < ind.size(); i++) {
    X_train.row(i) = X.row(ind[i]);
  }
  return X_train;
}

Eigen::MatrixXd extract_rows(const Eigen::MatrixXd &X,
                             Eigen::VectorXi &ind_train) {
  Eigen::MatrixXd X_train(ind_train.size(), X.cols());

  for (int i = 0; i < ind_train.size(); i++) {
    X_train.row(i) = X.row(ind_train(i));
  }
  return X_train;
}

vector<int> powercellidx(const Eigen::MatrixXd &robot_positions,
                         const Eigen::MatrixXd &Xss, const int &robot_id) {
  Eigen::MatrixXd Distance = pdist2(robot_positions, Xss);  // 945 x 3
  // cout<<"finish distance"<<endl;
  // cout<<"distance size"<<Distance.rows() <<"x"<< Distance.cols()<<endl;
  vector<int> q;
  Eigen::MatrixXd::Index min_index;
  for (int i = 0; i < Xss.rows(); i++) {
    Distance.col(i).minCoeff(&min_index);
    if (min_index == robot_id) q.push_back(i);
  }
  return q;
}

vector<vector<int>> batch_powercellidx(const Eigen::MatrixXd &robot_positions,
                                       const Eigen::MatrixXd &Xss) {
  /*
   * Input:
   *      g: 3 x 2   robot locations
   *      s: 945 x 2 Xss
   * Output:
   *      idx: 945 x 1
   */
  // cout<<"Robot positions "<<robot_positions.rows()<<"
  // "<<robot_positions.cols()<<endl;
  // cout<<"Xss "<<Xss.rows()<<" "<<Xss.cols()<<endl;
  int num_robots = robot_positions.rows();

  Eigen::MatrixXd Distance = pdist2(robot_positions, Xss);  // 945 x 3
  // cout<<"finish distance"<<endl;
  // cout<<"distance size"<<Distance.rows() <<"x"<< Distance.cols()<<endl;
  vector<vector<int>> ans;
  ans.resize(num_robots);
  Eigen::MatrixXd::Index min_index;
  for (int i = 0; i < Xss.rows(); i++) {
    Distance.col(i).minCoeff(&min_index);
    ans[min_index].push_back(i);
  }
  return ans;
}

Eigen::MatrixXd compute_utility_center(const int &robot_id,
                                       const Eigen::MatrixXd &rob_positions,
                                       const Eigen::MatrixXd &Xss,
                                       const Eigen::MatrixXd &phi) {
  // first get closest positions
  std::cout << "Enter Computer Utility Center" << std::endl;
  vector<int> q_index = powercellidx(rob_positions, Xss, robot_id);

  std::cout << "Finish powercell" << std::endl;
  // calculate voroni centers
  double c_qx = 0.0;
  double c_qy = 0.0;
  double c_q = 0.0;

  for (int i = 0; i < q_index.size(); i++) {
    c_qx += Xss(q_index[i], 0) * phi(q_index[i]);
    c_qy += Xss(q_index[i], 1) * phi(q_index[i]);
    c_q += phi(q_index[i]);
  }

  Eigen::MatrixXd v_center = Eigen::MatrixXd::Zero(1, 2);
  v_center(0, 0) = c_qx / c_q;
  v_center(0, 1) = c_qy / c_q;

  return v_center;
}

vector<int> sort_indexes(Eigen::MatrixXd &v) {
  // initialize original index locations
  vector<int> idx(v.cols());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) { return v(0, i1) < v(0, i2); });

  return idx;
}

bool load_ground_truth_data(const std::string &location_data_path,
                            const std::string &temperature_data_path,
                            Eigen::MatrixXd &location,
                            Eigen::MatrixXd &temperature) {
  ifstream finFss(temperature_data_path);

  if (!finFss.is_open()) {
    cout << "open Fss File: Error opening file" << endl;
    return false;
  }

  vector<double> Fss_vec;
  string line;

  while (getline(finFss, line)) {
    istringstream sin(line);

    string field;
    double a;

    while (getline(sin, field, ',')) {
      a = stod(field);
      Fss_vec.push_back(a);
    }
  }
  temperature.resize(Fss_vec.size(), 1);
  for (int i = 0; i < Fss_vec.size(); i++) {
    temperature(i, 0) = Fss_vec[i];
  }

  ifstream finXss(location_data_path);

  if (!finXss.is_open()) {
    cout << "open Xss File: Error opening file" << endl;
    return false;
  }

  vector<double> Xss_x;
  vector<double> Xss_y;
  string line1;

  while (getline(finXss, line1)) {
    istringstream sin(line1);

    string field;
    double a;
    int n = 0;

    while (getline(sin, field, ',')) {
      a = stod(field);
      if (n == 0) {
        Xss_x.push_back(a);
        n = n + 1;
      } else if (n == 1) {
        Xss_y.push_back(a);
      }
    }
  }
  location.resize(Xss_x.size(), 2);
  for (int i = 0; i < Xss_x.size(); i++) {
    location(i, 0) = Xss_x[i];
    location(i, 1) = Xss_y[i];
  }
}
}  // namespace sampling