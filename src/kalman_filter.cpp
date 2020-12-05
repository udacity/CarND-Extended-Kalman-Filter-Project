#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
                        MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
    * Predict the state.
  */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd& z) {
  /**
    * Update the state using Kalman Filter equations.
  */

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // New state and uncertainty covariance.
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
  /**
    * Update the state using Extended Kalman Filter equations
  */

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(px * px + py * py);
  float phi;
  float rho_dot;

  if (px < 0.0001 && py < 0.0001) {
    phi = 0;
  } else {
    phi = atan2(py, px);
  }

  if (rho > 0.0001) {
    rho_dot = (px * vx + py * vy) / rho;
  } else {
    rho_dot = 0;
  }

  // h(x) maps cartesian coordinates to polar coordinates.
  VectorXd hx(3);
  hx << rho, phi, rho_dot;

  VectorXd y = z - hx;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  const float PI = 3.1415927;

  /* In C++, atan2() returns values between -pi and pi.
   * When calculating phi in y = z - h(x) for radar measurements,
   * the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi.
   * The Kalman filter is expecting small angle values between the range -pi and pi.
   * HINT: when working in radians,
   * you can add 2π or subtract 2π until the angle is within the desired range. */

  while (y[1] > PI) {
    y[1] -= 2 * PI;
  }
  while (y[1] < -PI) {
    y[1] += 2 * PI;
  }

  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;

}
