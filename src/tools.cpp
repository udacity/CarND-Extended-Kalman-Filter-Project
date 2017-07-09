#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
 VectorXd rmse(4);
  rmse << 0,0,0,0;
  // Data validation.
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
	  //cout << "ERROR: No Values or missing Values!! ";
      return rmse;
  }
  // Evaluate Sum estimations - ground_truth ^2
  else {
    for (unsigned int i = 0; i < estimations.size(); i++){
	  VectorXd residual = estimations[i] - ground_truth[i];
	  residual = residual.array() * residual.array();
	  rmse += residual; 	
    }
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
  }
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
    // Calculate terms
	float Spx2py2 = sqrt((pow(px,2)+pow(py,2)));
	float px2py2 = (pow(px,2)+pow(py,2));
	float px2py2a32 =  sqrt(pow((pow(px,2)+pow(py,2)),3));
	float pyv = py*(vx*py-vy*px);
	float pxv = px*(vy*px-vx*py);
	// check division by zero
	if ((Spx2py2 || px2py2 || px2py2a32) == 0 ){
		//cout << "ERROR: Zero Division";
	}
	else {
	//compute the Jacobian matrix
	Hj << 	px / Spx2py2, py / Spx2py2, 0, 0,
			-(py / px2py2), px / px2py2, 0, 0,
			pyv / px2py2a32, pxv / px2py2a32, px / Spx2py2, py / Spx2py2;
	}
	return Hj;
}
