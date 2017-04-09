#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  H_laser_ << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  noise_ax_ = noise_ay_ = 9.0; // taken from hint/instruction in code below
  //ekf_.Init(VectorXd(4), MatrixXd(4, 4), MatrixXd(4, 4), MatrixXd(3, 4), R_laser_, MatrixXd(4, 4));
            //  x,          P,                 F,               H,           R,       Q
  ekf_.x_ = VectorXd(4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

  ekf_.x_ << 1.0, 1.0, 1.0, 1.0;
  ekf_.P_ <<
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;
  ekf_.F_ <<
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
  ekf_.Q_ << 
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::initialize(const MeasurementPackage &measurement_pack) {
  /**
  TODO:
  * Initialize the state ekf_.x_ with the first measurement.
  * Create the covariance matrix.
  * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */
  // first measurement
  cout << "EKF: " << endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  //ekf_.x_ = VectorXd(4);
  //ekf_.x_ << 1, 1, 1, 1;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Convert radar from polar to cartesian coordinates and initialize state.
    double rho = measurement_pack.raw_measurements_[0];
    double phi = measurement_pack.raw_measurements_[1];
    double rate = measurement_pack.raw_measurements_[2];

    ekf_.x_ << rho*cos(phi), rho*sin(phi), rate*cos(phi), rate*sin(phi);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    if (ekf_.x_[0] == 0.0) ekf_.x_[0] = 0.01;
    if (ekf_.x_[1] == 0.0) ekf_.x_[1] = 0.01;
  }

  is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    initialize(measurement_pack);
    return;   // done initializing, no need to predict or update
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;

  //2. Set the process covariance matrix Q
  //double dt2 = dt*dt, dt3 = dt*dt*dt / 2, dt4 = dt*dt*dt*dt / 4;
  //ekf_.Q_(0, 0) = dt4*noise_ax_;
  //ekf_.Q_(1, 1) = dt4*noise_ay_;
  //ekf_.Q_(2, 0) = ekf_.Q_(0, 2) = dt3*noise_ax_;
  //ekf_.Q_(3, 1) = ekf_.Q_(1, 3) = dt3*noise_ay_;
  //ekf_.Q_(2, 2) = dt2*noise_ax_;
  //ekf_.Q_(3, 3) = dt2*noise_ay_;

  double dt_2 = dt * dt; double dt_3 = dt_2 * dt; double dt_4 = dt_3 * dt;
  ekf_.Q_ <<  dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
              0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
              dt_3 / 2 * noise_ax_, 0, dt_2*noise_ax_, 0,
              0, dt_3 / 2 * noise_ay_, 0, dt_2*noise_ay_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    VectorXd z = VectorXd(3); 
    z[0] = measurement_pack.raw_measurements_[0];
    z[1] = measurement_pack.raw_measurements_[1];
    z[2] = measurement_pack.raw_measurements_[2];
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  } else {
    // Laser updates
    VectorXd z = VectorXd(2);
    z[0] = measurement_pack.raw_measurements_[0];
    z[1] = measurement_pack.raw_measurements_[1];
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
