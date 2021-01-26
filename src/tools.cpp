#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate RMSE : https://en.wikipedia.org/wiki/Root-mean-square_deviation
 *
 * @param estimations
 * @param ground_truth
 * @return : RMSE vector
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() or estimations.empty())
    {
        cout << "Invalid estimation or ground thruth data\n";
        return rmse;
    }

    // accumulate squared residuals
    for (size_t i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;

}

/**
 * Calculate Jacobian matrix : https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
 * @param x_state
 * @return : Jacobian matrix
 */
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{

    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;

    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        cout << "CalculationJacobian() - Error - Division by zero\n";
        return Hj;
    }

    // compute the Jacobian matrix

    Hj << (px / c2), (py / c2), 0, 0,
            -(py / c1), (px / c1), 0, 0,
            py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
