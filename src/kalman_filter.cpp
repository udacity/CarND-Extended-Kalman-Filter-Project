#include "kalman_filter.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;

KalmanFilter::KalmanFilter() : I (Eigen::Matrix4d::Identity()) {
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;
  Ht_laser_ = H_laser_.transpose();

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
    0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  Matrix4d Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Vector2d &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Vector2d z_pred{ H_laser_ * x_ };
  Vector2d y{ z - z_pred };
  //MatrixXd Ht = H_.transpose();
  Matrix2d S{ H_laser_ * P_ * Ht_laser_ + R_laser_ };
  Matrix2d Si{ S.inverse() };
  Eigen::Matrix<double, 4, 2> PHt{ P_ * Ht_laser_ };
  Eigen::Matrix<double, 4, 2> K{ PHt * Si };

  //new estimate
  x_ = x_ + (K * y);
  //size_t x_size = x_.size();
  //MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const Vector3d &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //VectorXd z_pred = H_ * x_;

  auto f1 = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  Vector3d z_pred{ f1, atan2(x_[1], x_[0]), (x_[0] * x_[2] + x_[1] * x_[3]) / f1 };
  Vector3d y{ z - z_pred };
  Htj_ = Hj_.transpose();
  Matrix3d S{ Hj_ * P_ * Htj_ + R_radar_ };
  Matrix3d Si{ S.inverse() };
  Eigen::Matrix<double, 4, 3> PHt{ P_ * Htj_ };
  Eigen::Matrix<double, 4, 3> K{ PHt * Si };

  //new estimate
  x_ = x_ + (K * y);
  //size_t x_size = x_.size();
  //MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}
