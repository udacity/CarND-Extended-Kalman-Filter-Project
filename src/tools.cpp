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

    //Validate parameters
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        __throw_invalid_argument("Invalid estimation vector size");
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd diff = estimations[i] - ground_truth[i];
        rmse += (diff.array() * diff.array()).matrix();
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //TODO: YOUR CODE HERE
    double px2py2 = px*px + py*py;
    //check division by zero/overflow
    if (px2py2 > 0.0001) {
        //compute the Jacobian matrix
        double sqr = sqrt(px2py2);
        double sqr3 = pow(sqr,3);
        double px_sum = px / px2py2;
        double py_sum = py / px2py2;
        double px_sqr = px / sqr;
        double py_sqr = py / sqr;
        Hj <<   px_sqr,                  py_sqr,                  0,      0,
                -py_sum,                 px_sum,                  0,      0,
                py*(vx*py - vy*px)/sqr3, px*(vy*px - vx*py)/sqr3, px_sqr, py_sqr;
    } else {
        __throw_overflow_error("Denominator too small");
    }

    return Hj;
}
