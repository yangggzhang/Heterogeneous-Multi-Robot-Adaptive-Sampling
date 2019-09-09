#include "gp_utils.h"
namespace sampling {

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

Eigen::MatrixXd loggausspdf(Eigen::MatrixXd X, Eigen::MatrixXd mu,
                            Eigen::MatrixXd Sigma) {
  int d = X.rows();
  int n = X.cols();
  for (int i = 0; i < d; i++) {
    for (int j = 0; j < n; j++) {
      X(i, j) = X(i, j) - mu(i, 0);  // mu: d x 1
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
      R(i, j) = R(i, j) + w.array().log()(0, j);  // w: 1 x k
    }
  }
  // cout<<"after 2: "<<R<<endl;

  Eigen::MatrixXd T = R.array().exp().rowwise().sum().log();  // T: n x 1
  double llh_iter = T.sum() / n;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < k; j++) {
      R(i, j) = R(i, j) - T(i, 0);  // w: 1 x k
    }
  }
  // cout<<"after 3: "<<R<<endl;

  R = R.array().exp();
  // cout<<"after: "<<R<<endl;

  return llh_iter;
}

void maximization(const Eigen::MatrixXd &X, Model &gpModel,
                  const Eigen::MatrixXd &R) {
  int d = X.rows();             // d : utility dimension
  int n = X.cols();             // n : number of samples
  int k = gpModel.numGaussian;  // k : number of gaussian

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
      Eigen::MatrixXd::Zero(d, d * k);  // use block to construct a 3D matrix,
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

}  // namespace sampling