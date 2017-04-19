#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate the Jacobian here.
   */
  MatrixXd Hj(3, 4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if ((px == 0) && (py == 0)) {
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
  }
  //compute the Jacobian matrix
  else {
    float px2_py2 = pow(px, 2) + pow(py, 2);

    Hj << px / sqrt(px2_py2), py / sqrt(px2_py2), 0, 0, -py / px2_py2, px
        / px2_py2, 0, 0, py * (vx * py - vy * px) / pow(px2_py2, 1.5), px
        * (vy * px - vx * py) / pow(px2_py2, 1.5), px / sqrt(px2_py2), py
        / sqrt(px2_py2);
  }

  return Hj;
}

VectorXd Tools::CalculateHprime(const VectorXd &x_state) {
  /**
   * Calculate the nonlinear transform here.
   */
  VectorXd x_prime(3);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
   if ((px == 0) && (py == 0)) {
     x_prime << 0, 0, 0;

     cout << "CalculateHprime() - Error - Division by Zero" << endl;
   }
   //compute the polar transform
   else
   {
     float range = sqrt(px*px + py*py);
     float bearing = atan2(py,px);
     float angular_velocity = (px*vx + py*vy)/range;

     x_prime << range, bearing, angular_velocity;
   }

   return x_prime;
}
