#include "kalman_filter.h"
#include <iostream>

#define TWO_PI (2.0*M_PI)

using namespace std;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_*x_;
  update_(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px2py2 = x_[0]*x_[0] + x_[1]*x_[1];
  if (abs(x_[0]) > 0.0001 && px2py2 > 0.0001) {
    double sqr = sqrt(px2py2);
    VectorXd z_pred = VectorXd(3);
    z_pred << sqr, atan2(x_[1], x_[0]), (x_[0]*x_[2] + x_[1]*x_[3])/sqr;
    VectorXd y = z - z_pred;
    //Normalize phi
    y[1] -= trunc(y[1]/TWO_PI) * TWO_PI;
    update_(y);
  } else {
    cout << "Skipping update to avoid overflow/division by zero" << endl;
  }
}

void KalmanFilter::update_(const Eigen::VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

