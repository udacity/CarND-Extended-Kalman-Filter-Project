#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
//#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::Vector4d CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  //Tools tools;

  // noise
  double noise_ax_;
  double noise_ay_;

  // initialize
  void initialize(const MeasurementPackage& measurement_pack);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::Matrix<double, 3, 4> CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif /* FusionEKF_H_ */
