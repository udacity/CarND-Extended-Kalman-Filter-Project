#include "kalman_filter.h"
#include <iostream>


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
  /**
  TODO:
    * predict the state
  */
  std::cout << "Predict " << std::endl;
  x_ = (F_ * x_);
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; 
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd I = MatrixXd::Identity(4, 4);
  MatrixXd Ht = H_.transpose();
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
  x_ = x_ + (K * y);
  P_ =  (I - (K * H_)) * P_;

  
  
  
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h_x_(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  float hx1 = sqrt((px * px) + (py * py));
  float hx2 = atan2(py, px);
  float hx3 = ((px * vx) + (py * vy))/ hx1;
  h_x_ << hx1, hx2, hx3; 
  
  VectorXd y = z - h_x_;
  std::cout << "y RADAR!!! " << y << std::endl;  
  

  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
  x_ = x_ + (K * y);
  P_ =  (I - (K * H_)) * P_;

  
}
