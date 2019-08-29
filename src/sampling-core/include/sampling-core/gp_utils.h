/**
* Utility functions for GP update
* AUTHOR: Yang Zhang
*/

#pragma once
// this format is needed for the voronoi header
#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"

#include "load_data.h"
#include <CentralizedController.h>
#include <cmuswarm_msgs/BehaviourRequest.h>
#include <cmuswarm_msgs/ObjectCoverageLocation.h>
#include <cmuswarm_msgs/ObjectCoverageLocations.h>
#include <cstdlib>
#include <cstdlib>
#include <fstream>
#include <std_srvs/Trigger.h>
#include <stdlib.h>
#include <string>

namespace sampling {

/// Inuput location_1 location_2
/// Output distane between sets of locations
/// Example :
/// Compute the euclidean distance between Xtest and Xs
/// Input: location_1: 10 x 2, location_2: 945 x 2
/// Output: Distance: 10 x 945
bool distance_2d(const Eigen::MatrixXd &location_1,
                 const Eigen::MatrixXd &location_2, Eigen::MatrixXd &distance);

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

  Eigen::MatrixXd Distance = pdist2(robot_positions, Xss); // 945 x 3
  // cout<<"finish distance"<<endl;
  // cout<<"distance size"<<Distance.rows() <<"x"<< Distance.cols()<<endl;
  vector<int> q;
  Eigen::MatrixXd::Index min_index;
  for (int i = 0; i < Xss.rows(); i++) {
    Distance.col(i).minCoeff(&min_index);
    if (min_index == robot_id)
      q.push_back(i);
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

  Eigen::MatrixXd Distance = pdist2(robot_positions, Xss); // 945 x 3
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

Eigen::MatrixXd loggausspdf(Eigen::MatrixXd X, Eigen::MatrixXd mu,
                            Eigen::MatrixXd Sigma) {
  int d = X.rows();
  int n = X.cols();
  for (int i = 0; i < d; i++) {
    for (int j = 0; j < n; j++) {
      X(i, j) = X(i, j) - mu(i, 0); // mu: d x 1
    }
  }
  Eigen::MatrixXd U = Sigma.llt().matrixL();
  Eigen::MatrixXd Q = U.inverse() * X;
  Eigen::MatrixXd q = Q.array() * Q.array();

  double c = d * log(2 * M_PI) + 2 * U.diagonal().array().log().sum();

  Eigen::MatrixXd y = -1 * (c + q.array()) / 2;
  return y.transpose();
}

double expectation(const Eigen::MatrixXd &X, Model &gpModel,
                   Eigen::MatrixXd &R) {
  // cout<<"before: "<<R<<endl;
  Eigen::MatrixXd mu = gpModel.mu;
  Eigen::MatrixXd Sigma = gpModel.Sigma;
  Eigen::MatrixXd w = gpModel.w;

  int d = X.rows();
  int n = X.cols();
  int k = mu.cols();

  for (int i = 0; i < k; i++) {
    R.col(i) = loggausspdf(X, mu.col(i), Sigma.block<1, 1>(0, i));
  }
  // cout<<"after 1: "<<R<<endl;

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < k; j++) {
      R(i, j) = R(i, j) + w.array().log()(0, j); // w: 1 x k
    }
  }
  // cout<<"after 2: "<<R<<endl;

  Eigen::MatrixXd T = R.array().exp().rowwise().sum().log(); // T: n x 1
  double llh_iter = T.sum() / n;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < k; j++) {
      R(i, j) = R(i, j) - T(i, 0); // w: 1 x k
    }
  }
  // cout<<"after 3: "<<R<<endl;

  R = R.array().exp();
  // cout<<"after: "<<R<<endl;

  return llh_iter;
}

void maximization(const Eigen::MatrixXd &X, Model &gpModel,
                  const Eigen::MatrixXd &R) {
  int d = X.rows();            // d : utility dimension
  int n = X.cols();            // n : number of samples
  int k = gpModel.numGaussian; // k : number of gaussian

  Eigen::MatrixXd nk = R.colwise().sum();
  Eigen::MatrixXd w = nk / n;
  Eigen::MatrixXd temp = X * R;
  Eigen::MatrixXd mu(d, k);

  for (int i = 0; i < d; i++) {
    for (int j = 0; j < k; j++) {
      mu(j) = temp(i, j) / nk(i, j);
    }
  }

  Eigen::MatrixXd Sigma =
      Eigen::MatrixXd::Zero(d, d * k); // use block to construct a 3D matrix,
                                       // leftCols(d) as a first element
  Eigen::MatrixXd r = R.array().sqrt();
  for (int m = 0; m < k; m++) {
    Eigen::MatrixXd Xo(d, n);
    for (int i = 0; i < d; i++) {
      for (int j = 0; j < n; j++) {
        Xo(i, j) = X(i, j) - mu(i, m);
      }
    }
    Xo = Xo.array() * r.col(m).transpose().array();
    Sigma.block<1, 1>(0, m) =
        Xo * Xo.transpose() / nk(m) + Eigen::MatrixXd::Identity(d, d) * (1e-20);
  }

  gpModel.mu = mu;
  gpModel.Sigma = Sigma;
  gpModel.w = w;
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

void CentralizedController::MixGaussEm_gmm() {
  /*Perform EM algorithm for fitting the Gaussian mixture model
  *Output: label: 1 x 945 cluster label
  * model: trained model structure
  * llh:loglikelihood
  */
  // bool break_flag = false;

  double inf = numeric_limits<double>::infinity();
  double last_llh = 0;
  double current_llh = 0;

  Eigen::MatrixXd::Index max_index;
  Fss.transposeInPlace();
  for (int iter = 0; iter < EM_MAX_ITER; iter++) {

    maximization(Fss, gpModel, R);
    current_llh = expectation(Fss, gpModel, R); // update llh[iter] and R
    // cout<<"last llh "<<last_llh<<endl;
    // cout<<"current llh "<<current_llh<<endl;

    if (abs(current_llh - last_llh) < EM_TOL * abs(current_llh))
      break;
    last_llh = current_llh;
  }

  Fss.transposeInPlace();
}

void CentralizedController::MixGaussPred_gmm() {
  /* Predict label and responsibility for Gaussian mixture model.
*Input:
*     X: d x n data matrix
*     model: trained model structure outputted by the EM algorithm
*Output:
*     label: 1 x n cluster label
*     R: k x n responsibility
*/
  Eigen::MatrixXd::Index max_index;
  for (int i = 0; i < R.rows(); i++) {
    R.row(i).maxCoeff(&max_index);
    label(i) = max_index; // Eigen::VectorXd label(X.cols());
  }
}

Eigen::VectorXi sort_unique(Eigen::VectorXi a) {
  sort(a.data(), a.data() + a.size(),
       [](int lhs, int rhs) { return rhs > lhs; });

  vector<int> vec;
  vec.clear();
  for (int i = 0; i < a.size(); i++)
    vec.push_back(a(i));
  vector<int>::iterator it;
  it = unique(vec.begin(), vec.end());
  vec.erase(it, vec.end());
  Eigen::VectorXi b(vec.size());
  for (int i = 0; i < vec.size(); i++) {
    b(i) = vec[i];
  }

  return b;
}

void gp_compute(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y,
                const Eigen::MatrixXd &Xtest, Eigen::VectorXd &mu,
                Eigen::VectorXd &s2) {
  double y;
  GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
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

void gpml_rms(Eigen::VectorXi &ind_train, const Eigen::MatrixXd &Xs,
              const Eigen::MatrixXd &Fs, const Eigen::MatrixXd &Xtest_new,
              Eigen::VectorXd &mu, Eigen::VectorXd &s2) {
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
  ind_train = sort_unique(ind_train);
  Eigen::MatrixXd Xs_train;
  Eigen::MatrixXd Fs_train;

  Xs_train = extract_rows(Xs, ind_train);
  Fs_train = extract_rows(Fs, ind_train);

  Eigen::MatrixXd Fs_train_mtz;
  Fs_train_mtz = Fs_train.array() - Fs_train.mean(); // mean value equals to 0
  gp_compute(Xs_train, Fs_train_mtz, Xtest_new, mu, s2);
  mu = mu.array() + Fs_train.mean();
}

Eigen::MatrixXd set_NaN_tz(Eigen::MatrixXd X) {
  /*
*set NaN elements to zero
*/
  for (int i = 0; i < X.rows(); i++) {
    for (int j = 0; j < X.cols(); j++) {
      if (isnan(X(i, j)) == 1) {
        X(i, j) = 0;
      }
    }
  }
  return X;
}

Eigen::MatrixXd repmat(Eigen::VectorXd X, int n) {
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

Eigen::VectorXi get_label_idx(Eigen::VectorXi k, int iij) {
  vector<int> idx_vec;
  for (int i = 0; i < k.size(); i++) {
    if (k(i) == iij) {
      idx_vec.push_back(i);
    }
  }
  if (idx_vec.size() > 0) {
    Eigen::VectorXi idx(idx_vec.size());
    for (int i = 0; i < idx.size(); i++) {
      idx(i) = idx_vec[i];
    }
    return idx;
  } else {
    Eigen::VectorXi idx(1);
    idx << -1;
    return idx;
  }
}

Eigen::MatrixXd gt_pred(Eigen::MatrixXd Xs, Eigen::MatrixXd R,
                        Eigen::MatrixXd X_test) {
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
  int K = R.cols();           // get number of models, 3
  int N = Xs.rows();          // 10
  int n_test = X_test.rows(); // 202

  Eigen::MatrixXd PP = Eigen::MatrixXd::Zero(n_test, K); // 202 x 3

  Eigen::VectorXi ind_train(N);
  for (int i = 0; i < N; i++) {
    ind_train(i) = i;
  }

  for (int ijk = 0; ijk < K; ijk++) {
    Eigen::VectorXd mu_gp(n_test);
    Eigen::VectorXd s2_gp(n_test);
    gpml_rms(ind_train, Xs, R.col(ijk), X_test, mu_gp, s2_gp);
    PP.col(ijk) = mu_gp;
  }

  Eigen::MatrixXd PP_out(n_test, K);
  for (int i = 0; i < PP_out.rows(); i++) {
    PP_out.row(i) = PP.row(i).array() / PP.row(i).sum();
  }
  return PP_out;
}

void CentralizedController::gmm_pred_cen(Eigen::VectorXd &pred_h,
                                         Eigen::VectorXd &pred_Var) {
  /*
   * Input:
   *      Xtest: 202 x 2, Ftest: 202 x 1
  */
  // Eigen::MatrixXd Xtrain -> Xss
  // Eigen::MatrixXd Ftrain -> Fss
  // Eigen::MatrixXd Xtest -> all gt Xss
  // Model model -> gpModel
  // gt_data just use num_of_gaussian

  // cout << "label is " << label << endl;

  Eigen::MatrixXd mu = Eigen::MatrixXd::Zero(
      All_Xss.rows(), gpModel.numGaussian); // Sample_size x 3
  Eigen::MatrixXd s2 = Eigen::MatrixXd::Zero(
      All_Xss.rows(), gpModel.numGaussian); // Sample_size x 3

  for (int ijk = 0; ijk < gpModel.numGaussian; ijk++) {
    // cout << "I start getting labels." << endl;
    Eigen::VectorXi ind_train = get_label_idx(label, ijk);
    // cout << "indices train is " << ind_train << endl;
    if (ind_train(0) == -1) {
      continue;
    }
    Eigen::VectorXd mu_vec(All_Xss.rows());
    Eigen::VectorXd s2_vec(All_Xss.rows());
    gpml_rms(ind_train, Xss, Fss, All_Xss, mu_vec, s2_vec);
    mu.col(ijk) = mu_vec;
    s2.col(ijk) = s2_vec;
  }

  Eigen::MatrixXd PP_out;
  PP_out = gt_pred(Xss, R, All_Xss);

  /*
*start to filter infeasible component
*/

  Eigen::MatrixXd pred_mu_mat = mu; // 202 x 3
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

  pred_mu_mat = set_NaN_tz(pred_mu_mat); // set NaN elements to 0
                                         /*
                                         * end of filtering
                                         */
  // Eigen::MatrixXd pred_h = (norm_PP_out.array() *
  // pred_mu_mat.array()).rowwise().sum();//202 x 1
  pred_h =
      (norm_PP_out.array() * pred_mu_mat.array()).rowwise().sum(); // 202 x 1

  Eigen::MatrixXd pred_s2_mat = s2;

  Eigen::MatrixXd muu_pp_rep = repmat(pred_h, gpModel.numGaussian); // 202 x 3
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

  // return ucb_pq;
}

// void CentralizedController::update_targets(priority_queue< pair<double,int>,
// vector<pair<double,int>>, less<pair<double,int>>> &ucb_pq)
// {
//     for (int i = 0; i < DEFAULT_REQUEST_BUTTER_SIZE; i++)
//     {
//         pair<double,int> target_pair = ucb_pq.top();
//         int target_id = target_pair.second;
//         ucb_pq.pop();

//         target_positions[i] =
//         make_pair(All_Xss(target_id,0),All_Xss(target_id,1));
//     }
// }

void CentralizedController::sample_cb(const cmuswarm_msgs::Sample &msg) {
  // cout<<"Master listened !!!"<<endl;
  int id = msg.robot_id;
  int pos_x = msg.x / CELL_SIZE;
  int pos_y = msg.y / CELL_SIZE;
  Eigen::MatrixXd position = Eigen::MatrixXd::Zero(1, 2);
  position(0, 0) = pos_x;
  position(0, 1) = pos_y;

  this->collectCount[position]++;

  // Eigen::MatrixXd X_utility; // collected_temperatures
  // unordered_map<Eigen::MatrixXd,int,EigenHash> collectedSamples; // using
  // unordered map to record all collected maps's position in X_utility, make
  // update for existing sample easies

  // for now grab tempterature from ground truth
  double utility = this->gt_data.gtSamples[position];

  if (collectedSamples.count(position)) // position collected before
  {
    int pos_id = collectedSamples[position];
    Fss(id, 0) = utility;
  } else {
    // not colelcted before
    // 1. enlarge matrix size (Xss Fss Model.R Model.label) 2. record id
    int len = Fss.rows();
    collectedSamples[position] = len;

    Fss.conservativeResize(len + 1, 1);
    Xss.conservativeResize(len + 1, 2);
    Xss(len, 0) = pos_x;
    Xss(len, 1) = pos_y;
    Fss(len, 0) = utility;

    label.conservativeResize(len + 1);
    R.conservativeResize(len + 1, gpModel.numGaussian);

    int rand_id = rand() % gpModel.numGaussian;

    double rand_r = 1.0 / (double)gpModel.numGaussian;

    for (int i = 0; i < gpModel.numGaussian; i++) {
      R(len, i) = rand_r;
      R(len, i) = rand() % 10;
      // if (i == rand_id) R(len,i) = 1;
      // else R(len,i) = 0;
    }
    R.row(len) = R.row(len) / R.row(len).sum();
    label(len) = rand_id;
  }

  // to be updated:
  // double utility = msg.utility

  // position(0,0) = pos_x;
  // position(0,1) = pos_y;

  // // for now use GT data
  //

  // if (!collectedSamples.count(position)) // new sample -> initial_lize R
  // {
  //     int rand_id = rand()%this->numBots;
  //     this->gpModel.R[position] = vector<double>(this->numBots,0.0);
  //     this->gpModel.R[position][rand_id] = 1.0;
  // }

  this->UpdateCount++;
  if (this->UpdateCount >= this->EMUpdateRate)
    this->updateFlag = 1;
  // double uti = msg.utility;
  stringstream ss;
  ss << "Master listened: Robot id : " << msg.robot_id
     << " Position : " << pos_x << " , " << pos_y
     << " Temperature : " << utility;
  string info = ss.str();
  cout << info << endl;
}

bool CentralizedController::assign_target(
    cmuswarm_msgs::RequestTarget::Request &req,
    cmuswarm_msgs::RequestTarget::Response &res) {
  int id = req.robot_id;
  if (req.ask_target == 1) {
    if (this->request_count[id] < this->initialPoints[id].size()) {
      res.x = this->initialPoints[id][this->request_count[id]](0, 0);
      res.y = this->initialPoints[id][this->request_count[id]](0, 1);
      this->request_count[id]++;
    } else {
      if (!this->robot_targets[id].empty()) {
        pair<double, int> key = this->robot_targets[id].top();
        this->robot_targets[id].pop();
        Eigen::MatrixXd next_target = this->All_Xss.row(key.second);
        res.x = next_target(0, 0);
        res.y = next_target(0, 1);
      } else {
        res.x = req.current_x;
        res.y = req.current_y;
      }
      this->total_request_count++; // might be a problem
    }
    this->target_positions[id] = make_pair(res.x, res.y);
  }

  if (this->total_request_count >= DEFAULT_REQUEST_BUTTER_SIZE)
    this->updateFlag = true;

  return true;
}

bool CentralizedController::check_reception() {
  for (int i = 0; i < numBots; i++) {
    if (!position_received_v[i])
      return false;
  }
  return true;
}

void CentralizedController::receive_positions(
    const cmuswarm_msgs::Position &msg) {
  position_received_v[msg.robot_id] = true;
  this->rob_positions(msg.robot_id, 0) = msg.x;
  this->rob_positions(msg.robot_id, 1) = msg.y;
}

void CentralizedController::request_positions() {
  for (int i = 0; i < numBots; i++) {
    cmuswarm_msgs::PositionRequest msg;
    msg.robot_id = i;
    msg.request = 1;
    this->position_request_pub_v[i].publish(msg);
    position_received_v[i] = false;
  }
}

void CentralizedController::update_targets() {
  // update distributed robots' locations
  // cout<<"Ask for positions!"<<endl;
  request_positions();
  while (true) {
    ros::spinOnce();
    if (check_reception())
      break;
  }
  // cout<<"Received all positions!"<<endl;
  // Collected all robots' locations

  vector<vector<int>> q =
      batch_powercellidx(this->rob_positions, this->All_Xss);
  // cout<<" batch powercellidx"<<endl;

  for (int i = 0; i < q.size(); i++) {
    //    vector<priority_queue< pair<double,int>, vector<pair<double,int>>,
    //    less<pair<double,int> > > robot_targets;
    robot_targets[i] =
        priority_queue<pair<double, int>, vector<pair<double, int>>,
                       less<pair<double, int>>>();

    Eigen::MatrixXd close_Xss =
        extract_rows(this->All_Xss, q[i]); // closes points

    for (int j = 0; j < q[i].size(); j++) {
      // calculate distance
      // Eigen::MatrixXd ego_position = All_Xss.row(q[i][j]);

      // Eigen::MatrixXd p_dist = pdist2(ego_position, close_Xss);
      // // multiply by utility
      // double utility = 0.0;
      // for (int k = 0; k < q[i].size(); k++)
      // {
      //     utility += p_dist(0,k) * this->phi(q[i][k]);
      // }

      robot_targets[i].push(make_pair(this->phi(q[i][j]), q[i][j]));
    }
    cout << "Finish update targets" << endl;
  }
}

CentralizedController::CentralizedController(const std::string &master_name_in,
                                             const std::string &num_in) {
  numBots = stoi(num_in);
  numGau = numBots;
  master_name = master_name_in;

  this->gpModel.numGaussian = numBots;

  // subscribe channels
  MasterHandle_v.resize(numBots);
  this->sample_info_sub_v.resize(numBots);
  this->position_request_pub_v.resize(numBots);
  this->position_info_sub_v.resize(numBots);
  this->rob_positions = Eigen::MatrixXd::Zero(numBots, 2);
  this->position_received_v = vector<bool>(numBots, false);
  this->pred_h = Eigen::VectorXd::Zero(945);
  this->pred_Var = Eigen::VectorXd::Zero(954);
  this->target_server_v.resize(numBots);

  for (int i = 0; i < numBots; i++) {
    ros::NodeHandle MasterHandle("");
    this->MasterHandle_v[i] = MasterHandle;

    string sample_topicname = "/swarmbot" + to_string(i) + "/collectedsamples";
    cout << "Subscribe : " << sample_topicname << endl;
    this->sample_info_sub_v[i] =
        MasterHandle.subscribe(sample_topicname, MAX_QUEUE_SIZE,
                               &CentralizedController::sample_cb, this);

    string target_topicname = "/swarmbot" + to_string(i) + "/update_targets";
    this->target_server_v[i] = MasterHandle.advertiseService(
        target_topicname, &CentralizedController::assign_target, this);

    string position_request_name =
        "/swarmbot" + to_string(i) + "/position_request";
    this->position_request_pub_v[i] =
        MasterHandle.advertise<cmuswarm_msgs::PositionRequest>(
            position_request_name, MAX_QUEUE_SIZE);

    string position_info_name = "/swarmbot" + to_string(i) + "/position_info";
    this->position_info_sub_v[i] =
        MasterHandle.subscribe(position_info_name, MAX_QUEUE_SIZE,
                               &CentralizedController::receive_positions, this);
  }

  // load ground truth
  this->gt_data.num_gau = DEFAULT_GAUSSIAN_NUM;
  this->gt_data.num_bot = numBots;
  this->gt_data.Xss = Eigen::MatrixXd::Zero(GT_DATASIZE, 2);
  this->gt_data.Fss = Eigen::MatrixXd::Zero(GT_DATASIZE, 1);
  load_data(this->gt_data.Xss, this->gt_data.Fss);
  this->All_Xss = this->gt_data.Xss;

  // initialize sample count
  for (int i = 0; i < this->All_Xss.rows(); i++) {
    this->collectCount[All_Xss.row(i)] = 1;
  }

  for (int i = 0; i < GT_DATASIZE; i++) {
    double x = this->gt_data.Xss(i, 0);
    double y = this->gt_data.Xss(i, 1);
    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(1, 2);
    temp(0, 0) = x;
    temp(0, 1) = y;
    // cout<<"temperature : "<<gt_data.Fss(i,0)<<endl;
    this->gt_data.gtSamples[temp] = gt_data.Fss(i, 0);
  }

  std::cout << "Finish reading gt data" << std::endl;

  // initialize EM
  this->phi = Eigen::VectorXd::Zero(GT_DATASIZE);
  this->EMUpdateRate = DEFALUT_EM_UPDATERATE;
  this->UpdateCount = 0;
  this->updateFlag = 0;
  this->total_request_count = 0;
  this->request_count = vector<int>(this->numBots, 0);
  initialPoints.resize(this->numBots);

  this->robot_targets.resize(this->numBots);

  // this->target_positions.resize(DEFAULT_REQUEST_BUTTER_SIZE);
  int X_step = MAP_X / (this->numBots + 1);
  // int Y_step = MAP_Y / (this->numBots + 1);

  // initialize samples
  for (int i = 0; i < this->numBots; i++) {
    int x_pos = (i + 1) * X_step;
    // if (i % 2 == 0)
    // {
    for (int j = 1; j < MAP_Y; j += 5) {
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(1, 2);
      temp(0, 0) = x_pos;
      temp(0, 1) = j;
      this->initialPoints[i].push_back(temp);
    }
    // }
    // else
    // {
    //     for (int j = MAP_Y - 1; j > 0; j -= 5)
    //     {
    //         Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(1,2);
    //         temp(0,0) = x_pos;
    //         temp(0,1) = j;
    //         this->initialPoints[i].push_back(temp);
    //     }
    // }
  }

  this->target_positions.resize(numBots);

  // stuff added for visualization.
  ros::NodeHandle global("");
  this->vor_pub = global.advertise<visualization_msgs::Marker>(
      "visualization_marker", MAX_QUEUE_SIZE);
  this->vor_init = false;
  // rb_visualization(gt_data.Fss, this->pred_h);
}

void CentralizedController::start() {
  cout << "Start listening to samples!!!" << endl;
  ros::Rate r(DEFAULT_RATE);
  int i = 1;
  while (ros::ok()) {
    if (this->updateFlag) {
      cout << "num_iter: " << i << endl;

      this->total_request_count = 0;

      // run EM;
      cout << "Start update EM" << endl;
      MixGaussEm_gmm(); // update EM
      cout << "Finish EM" << endl;
      MixGaussPred_gmm(); // update label
      cout << "Finish Predict" << endl;

      gmm_pred_cen(this->pred_h, this->pred_Var); // update phi function
      cout << "Finish GP" << endl;
      // cout<<"max: "<<this->pred_h.maxCoeff()<<" min:
      // "<<this->pred_h.minCoeff()<<endl;
      update_targets();

      // compute model rms error
      float rms =
          sqrt((this->gt_data.Fss - this->pred_h).array().pow(2).mean());
      cout << "abs error max: "
           << (this->gt_data.Fss - this->pred_h).array().abs().maxCoeff()
           << endl;
      cout << "abs error min: "
           << (this->gt_data.Fss - this->pred_h).array().abs().minCoeff()
           << endl;
      cout << "model rms error: " << rms << endl;
      cout << "avg variance: " << this->pred_Var.mean() << endl;

      // According to GP_prediction and voronoi cell to update targets
      // update_targets(ucb_pq);
      // cout<<"Finish update targets"<<endl;
      // update Priority_Queue
      this->updateFlag = 0;
      this->UpdateCount = 0;

      // update map in rviz
      rb_visualization(gt_data.Fss, this->pred_h, this->pred_Var);
      cout << "finish visualization" << endl;

      // break out when avg variance < 1
      if (this->pred_Var.mean() < 1) {
        cout << "=============== Model Converged ==============" << endl;
        cout << "After " << i << " iterations" << endl;
        // float rms = sqrt((this->gt_data.Fss -
        // this->pred_h).array().pow(2).mean());
        cout << "abs error max: "
             << (this->gt_data.Fss - this->pred_h).array().abs().maxCoeff()
             << endl;
        cout << "abs error min: "
             << (this->gt_data.Fss - this->pred_h).array().abs().minCoeff()
             << endl;
        cout << "model rms error: " << rms << endl;
        cout << "avg variance: " << this->pred_Var.mean() << endl;
        break;
      }

      i++;
    }
    ros::spinOnce();
    r.sleep();
  }
}

// =========================== Visualization ==========================//
// This function takes an nX2 eigen matrix with n number of seed points and
// returns a nX2 eigen matrix with all the voronoi lines
// The int num_lines is going to tell us how many lines were generated
Eigen::MatrixXd CentralizedController::generateVoronoiEdges(
    const Eigen::Ref<const Eigen::MatrixXd> &seedPoints, int num_points,
    int &num_lines) {
  jcv_rect bounding_box = {{0.0f, 0.0f}, {20.0f, 44.0f}};
  jcv_diagram diagram;
  const jcv_site *sites;
  jcv_graphedge *graph_edge;
  memset(&diagram, 0, sizeof(jcv_diagram));
  jcv_point points[num_points];

  for (int i = 0; i < num_points; i++) {
    points[i].x = seedPoints(i, 0);
    points[i].y = seedPoints(i, 1);
  }

  jcv_diagram_generate(num_points, (const jcv_point *)points, &bounding_box,
                       &diagram);

  sites = jcv_diagram_get_sites(&diagram);

  // just want to find out how many vales there are
  num_lines = 0;
  for (int i = 0; i < diagram.numsites; i++) {
    graph_edge = sites[i].edges;
    while (graph_edge) {
      num_lines += 2;
      graph_edge = graph_edge->next;
    }
  }

  Eigen::MatrixXd vorEdges(num_lines, 2);
  int j = 0;
  for (int i = 0; i < diagram.numsites; i++) {
    graph_edge = sites[i].edges;
    while (graph_edge) {
      vorEdges(j, 0) = graph_edge->pos[0].x;
      vorEdges(j, 1) = graph_edge->pos[0].y;
      vorEdges(j + 1, 0) = graph_edge->pos[1].x;
      vorEdges(j + 1, 1) = graph_edge->pos[1].y;
      j += 2;
      graph_edge = graph_edge->next;
    }
  }
  jcv_diagram_free(&diagram);
  return vorEdges;
}
}