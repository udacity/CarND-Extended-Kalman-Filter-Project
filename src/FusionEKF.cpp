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
  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
    * initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //create a 4D state vector, we don't know yet the values of the x state
  VectorXd x_in = VectorXd(4);

  //state covariance matrix P
  MatrixXd P_in = MatrixXd(4, 4);
  P_in << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  //the initial transition matrix F_
  MatrixXd F_in = MatrixXd(4, 4);
  F_in << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //the process covariance matrix Q
  MatrixXd Q_in = MatrixXd(4, 4);

  //Initialize Kalman Filter
  ekf_ = KalmanFilter();
  ekf_.Init(x_in,P_in,F_in,H_laser_,R_laser_,R_radar_,Q_in);

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /**
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to Cartesian coordinates.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
//      float ro_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_ << ro * cos(phi), ro * sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }


    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    //cout << "Kalman Filter Initialization " << endl;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  MatrixXd Q_in = MatrixXd(4, 4);
  Q_in <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Q_ = Q_in;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar measurement updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser measurement updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
}
