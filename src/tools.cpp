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

   MatrixXd Hj(3, 4);
   Hj.fill(0.0);
   float epslon = 0.00001;

   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   float c1 = px * px + py * py;
   float c2 = sqrt(c1);
   float c3 = (c1 * c2);

   // check division by zero
   if (fabs(c1) < epslon)
   {
      return Hj;
   }

   // compute the Jacobian matrix
   Hj << (px / c2), (py / c2), 0, 0,
       -(py / c1), (px / c1), 0, 0,
       py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

   return Hj;
}
