#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Size mis-match";
    return rmse;
  }
  
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd diff = (estimations[i] - ground_truth[i]).array();
    diff = diff * diff;
    rmse = rmse + diff;
  }
  
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //TODO: OPTIMIZE This later
  float denom = px * px + py * py;
  if (denom == 0) {
    return Hj;
  }
  //check division by zero
  
  //compute the Jacobian matrix
  Hj <<   px / sqrt(denom), py/sqrt(denom), 0, 0,
  -1 * py / denom, px/denom, 0, 0,
  py*(vx*py-vy*px)/pow(denom, 1.5), px*(vy*px-vx*py)/pow(denom, 1.5), px/sqrt(denom), py/sqrt(denom);
  
  return Hj;
}
