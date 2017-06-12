#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	if(estimations.size() == 0 || ground_truth.size() != estimations.size()) {
	    cout << "Big problems" << endl;
	    return rmse;
	}

	// Accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
    	VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
	}

	// Calculate the mean
	rmse /= estimations.size();

	// Calculate the squared root
	rmse = rmse.array().sqrt();

	// Return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
