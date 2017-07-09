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
  // Matrix F_ from FusionEKF
  // Calculation of x_  Predicted vector
  // Calculation of P_ Covariance Matrix.
  x_ = (F_ * x_);
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // z Vector from Laser measurement.
  // H_ Matrix specific for Laser defined on  FusionEKF.
  // R_ Matrix specific for Laser Measurement error, defined on FusionEKF. 
  MatrixXd I = MatrixXd::Identity(4, 4);
  MatrixXd Ht = H_.transpose();
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
  // Updated x_ Vector and P_ Matrix
  x_ = x_ + (K * y);
  P_ =  (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
   **/
  // Vector h_x_ to capture Radar Measurements. 
  VectorXd h_x_(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float hx1 = sqrt((px * px) + (py * py));
  float hx2 = 0.0001;
  if (fabs(x_[0]) > 0.0001) {
	  hx2 = atan2(py, px);  
  }
  // Avoid zero division.
  float hx3 = 0.0001;
  if (fabs(hx1) > 0.0001) {
	  hx3 = ((px * vx) + (py * vy))/ hx1;
  }
  h_x_ << hx1, hx2, hx3; 
  VectorXd y = z - h_x_;
  // Angle normalization. 
  if (y[1] > M_PI) {
	while (y[1] > M_PI) {
      y[1] -= 2*M_PI;
    }
  } else if (y[1] < -M_PI) {
    while (y[1] < -M_PI) {
      y[1] += 2*M_PI;
    }
  }
  // H_ Matrix Jaconian for Radar defined on  FusionEKF.
  // R_ Matrix specific for Radar Measurement error, defined on FusionEKF. 
  MatrixXd I = MatrixXd::Identity(4, 4); 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
  // Updated x_ Vector and P_ Matrix
  x_ = x_ + (K * y);
  P_ =  (I - (K * H_)) * P_;

}
