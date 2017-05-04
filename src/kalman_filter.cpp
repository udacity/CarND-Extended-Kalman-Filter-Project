#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  
  x_ = x_ + K * y;
  MatrixXd KH = K * H_;
  MatrixXd I = MatrixXd::Identity(KH.rows(), KH.cols());
  
  P_ = (I - KH) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::VectorXd &zest) {
  VectorXd y = z - zest;
  double phi = y(1);
  y(1) = tools.normalizeAngle(phi);

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  
  x_ = x_ + K * y;
  MatrixXd KH = K * H_;
  MatrixXd I = MatrixXd::Identity(KH.rows(), KH.cols());
  
  P_ = (I - KH) * P_;
}
