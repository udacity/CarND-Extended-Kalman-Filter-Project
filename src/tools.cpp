#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float h1 = px * px + py * py;
  float h2 = sqrt(h1);
  float h3 = vx * py - vy * vx;

  if (fabs(h1) < 0.0001) {
    Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    return Hj;
  }

  Hj << (px / h2), (py / h2), 0, 0,
      -(py / h1), (px / h1), 0, 0,
      py * h3 / (h1 * h2), -(px * h3 / (h1 * h2)), px / h2, py / h2;

  return Hj;

}

VectorXd Tools::CalculateRMSEContinuous(const Eigen::VectorXd& estimations,
                                        const Eigen::VectorXd& ground_truth,
                                        const Eigen::VectorXd& rmse,
                                        const int message_count) {
  /**
  * Update the RMSE with new measurement and ground truth.
  */
  VectorXd residual = estimations - ground_truth;
  residual = residual.array() * residual.array();
  VectorXd sum = rmse.array() * rmse.array() * message_count + residual.array();

  return (sum.array() / (message_count + 1)).sqrt();
}
