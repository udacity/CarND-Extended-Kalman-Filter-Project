#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

  // Predict by multiplying state transition function by x
  x_ = F_ * x_;

  // Update covariance as well
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // Calculate. y = z − H x
  VectorXd y = z - H_ * x_;

  // Update
  CommonKFUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Calculate Rho, Theta, RhoDot, for h vector
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double theta = std::atan2(x_(1), x_(0));
  double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;

  // Calculate y  
  VectorXd y = z - h;
  y(1) = std::atan2(sin(y(1)), cos(y(1)));

  // Update
  CommonKFUpdate(y);
}

void KalmanFilter::CommonKFUpdate(const VectorXd &y) {

  // S = H P Ht + R
  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  // K = P Ht Sinverse
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // Make our new estimate. x = x + K y
  x_ = x_ + (K * y);

  // P = (I − K H) P
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
