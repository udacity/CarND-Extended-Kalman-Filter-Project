#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
   /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd RMSE(ground_truth[0].size());
   RMSE.fill(0.0);
   size_t n = ground_truth.size();
   for (size_t i = 0; i < n; ++i)
   {
      // squared errors
      RMSE += (estimations[i] - ground_truth[i]) * (estimations[i] - ground_truth[i]);
   }
   RMSE /= n;
   RMSE = RMSE.array().sqrt();

   return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
   /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
