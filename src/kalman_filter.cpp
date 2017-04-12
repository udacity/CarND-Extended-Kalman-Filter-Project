#include "kalman_filter.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix;

KalmanFilter::KalmanFilter() : I (Matrix4d::Identity()) {
  x_ = VectorXd(4);   // state vector
  x_ << 1.0, 1.0, 1.0, 1.0;

  // Set the process and measurement noises
  P_ << // state covariance matrix
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;
  F_ << // state transistion matrix
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
  Q_ << // process covariance matrix (/noise/stochatic/random/assumed normally distributed)
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;

  H_laser_ << // measurement mapper - laser
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;
  Ht_laser_ = H_laser_.transpose();

  R_laser_ << //measurement covariance matrix - laser
    0.0225, 0.0,
    0.0,    0.0225;
  R_radar_ << //measurement covariance matrix - radar
    0.09, 0.0,    0.0,
    0.0,  0.0009, 0.0,
    0.0,  0.0,    0.09;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_      = F_ * x_;
  auto Ft = F_.transpose();
  P_      = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Vector2d &z) {
  // update the state by using Kalman Filter equations
  auto z_pred = H_laser_ * x_;
  auto y      = z - z_pred;
  auto S      = H_laser_ * P_ * Ht_laser_ + R_laser_;
  auto Si     = S.inverse();
  auto PHt    = P_ * Ht_laser_;
  auto K      = PHt * Si ;

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const Vector3d &z) {
  // update the state by using Extended Kalman Filter equations
  auto f1     = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  auto z_pred = Vector3d{ f1, atan2(x_[1], x_[0]), (x_[0] * x_[2] + x_[1] * x_[3]) / f1 }; // polar coords
  auto y      = z - z_pred;
  auto Htj    = Hj_.transpose();
  auto S      = Hj_ * P_ * Htj + R_radar_;
  auto Si     = S.inverse();
  auto PHt    = P_ * Htj ;
  auto K      = PHt * Si ;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * Hj_) * P_;
}
