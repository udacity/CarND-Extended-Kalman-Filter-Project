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

    // State prediction
    x_ = F_*x_;
    P_ = F_*P_*(F_.transpose()) + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {

    // State update for Laser
    MatrixXd I = MatrixXd::Identity(4, 4);
    
    VectorXd y_(2);
    y_ = z - H_*x_;
    
    MatrixXd S_(2,2);
    S_ = H_*P_*(H_.transpose()) + R_;
    
    MatrixXd K_(4,2);
    K_ = P_*(H_.transpose())*(S_.inverse());
    
    x_ = x_ + K_*y_;
    P_ = (I - K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
 
    // State update Radar
    MatrixXd I = MatrixXd::Identity(4, 4);
    
    VectorXd hx_;
    hx_ = VectorXd(3);
    hx_ << (pow((pow(x_(0),2)+pow(x_(1),2)),.5)),atan((x_(1)/x_(0))), (x_(0)*x_(2) + x_(1)*x_(3))/(pow((pow(x_(0),2)+pow(x_(1),2)),.5));
    
    VectorXd y_(3);
    y_ << z - hx_;
    
    MatrixXd S_(3,3);
    S_ << H_*P_*(H_.transpose()) + R_;
    
    MatrixXd K_(4,3);
    K_ << P_*(H_.transpose())*(S_.inverse());
    
    if(y_(1)>3.14) {y_(1) = y_(1)-6.28;}
    if(y_(1)<-3.14) {y_(1) = y_(1)+6.28;}
    
    //std::cout <<"Angles z = " << z(1) << " Angles hx" << hx_(1) << std::endl;
    //std::cout <<"x 1 = " << x_(1) << "x 2 = " << x_(0) << std::endl;
    //std::cout <<"z 1 = " << z(0)*sin(z(1)) << "x 2 = " << z(0)*cos(z(1)) << std::endl;
    
    x_ = x_ + K_*y_;
    P_ = (I - K_*H_)*P_;

}
