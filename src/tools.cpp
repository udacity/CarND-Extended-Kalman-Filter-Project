#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
    
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    VectorXd temp(4);
    VectorXd rtemp(4);
    rtemp << 0,0,0,0;
    int n = 0;
    
    for(int i = 0; i < estimations.size(); i++){
        temp = estimations[i] - ground_truth[i];
        for(int i = 0; i < temp.size(); i++){
            rtemp(i) = temp(i)*temp(i) +rtemp(i);
        }
        n++;
    }
    
    for(int i = 0; i < temp.size(); i++){
        rtemp(i) = pow((rtemp(i)/n),.5);
    }
    
    return rtemp;
    
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float pxy = px*px +py*py;

    if(pxy<.01){
        Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    
    }
    else{
        Hj << px/pow(pxy,.5), py/pow(pxy,.5), 0, 0,
        -py/pxy       , px/pxy        , 0, 0,
        py*(vx*py-vy*px)/pow(pxy,1.5), px*(-vx*py+vy*px)/pow(pxy,1.5), px/pow(pxy,.5), py/pow(pxy,.5);
    }
    
    return Hj;
    
}
