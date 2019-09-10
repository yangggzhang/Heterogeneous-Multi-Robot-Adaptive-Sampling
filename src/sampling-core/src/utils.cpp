#include "utils.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace sampling {
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
}