#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

using Eigen::VectorXd;
using Eigen::Vector4d;
using std::vector;

class FusionEKF {
public:
  FusionEKF();
  virtual ~FusionEKF();

  static constexpr double MIN_VAL = 0.01; /// minimum value to overcome divide by zero

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * A helper method to calculate RMSE.
  */
  Vector4d CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  void initialize(const MeasurementPackage& measurement_pack);
  Matrix<double, 3, 4> CalculateJacobian(const VectorXd& x_state); // A helper method to calculate Jacobians.

  bool is_initialized_;   // check whether the tracking toolbox was initiallized or not (first measurement)
  long long previous_timestamp_;

  // noise
  double noise_ax_;
  double noise_ay_;
};

#endif /* FusionEKF_H_ */
