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
   **/
  VectorXd h_x_(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  std::cout << "x_ Car to Pol " << x_ << std::endl;
  float hx1 = sqrt((px * px) + (py * py));
  std::cout << "hx1 " << hx1 << std::endl;
  float hx2 = 0.0001;
  if (fabs(x_[0]) > 0.0001) {
	  hx2 = atan2(py, px);
  std::cout << "hx2 " << hx2 << std::endl;	  
  }
  float hx3 = 0.0001;
  if (fabs(hx1) > 0.0001) {
	  hx3 = ((px * vx) + (py * vy))/ hx1;
  }
  std::cout << "hx3 " << hx3 << std::endl;
  h_x_ << hx1, hx2, hx3; 
  std::cout << "z  " << z << std::endl;
  VectorXd y = z - h_x_;
  std::cout << "y 1 " << y << std::endl;
 
if (y[1] > M_PI) {
	while (y[1] > M_PI) {
      y[1] -= 2*M_PI;
    }
} else if (y[1] < -M_PI) {
    while (y[1] < -M_PI) {
      y[1] += 2*M_PI;
    }
}    

  MatrixXd I = MatrixXd::Identity(4, 4); 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;
	// Updated State
  std::cout << "x_ " << x_ << std::endl;
  std::cout << "K " << K << std::endl;
  std::cout << "y 2" << y << std::endl;	
  x_ = x_ + (K * y);
  std::cout << "x_n " << x_ << std::endl;
  P_ =  (I - (K * H_)) * P_;

}
