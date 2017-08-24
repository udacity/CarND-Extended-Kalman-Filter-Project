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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // H Matrix initialized for Laser (remains the same throughout)
    H_laser_<< 1,0,0,0,
                0,1,0,0;
    
    // Kalman Filter Object Created
    ekf_ = KalmanFilter();
    
    // State Matrix Initialized. The Delta Time values are set as .05
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1,0,.05,0,
    0,1,0,.05,
    0,0,1,0,
    0,0,0,1;
    
    // Prediction Matrix Initialized
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1000,0,0,0,
    0,1000,0,0,
    0,0,1000,0,
    0,0,0,1000;
    
    // Process Matrix Initialized
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  
  if (!is_initialized_) {
      previous_timestamp_ = measurement_pack.timestamp_;
    
    // first measurement
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        ekf_.x_ << measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),0,0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

    // Process noise parameters for Process covariance matrix Q
    int noise_ax = 9;
    int noise_ay = 9;
    int q1 = noise_ay^2;
    int q2 = noise_ax^2;
    
    // Delta Time determined
    float dT = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // State matrix updated with time passed since last prediction
    ekf_.F_.row(0)(2) = dT;
    ekf_.F_.row(1)(3) = dT;
    
    // Process noise Matrix updated
    ekf_.Q_ << pow(dT,4)*q2/4, 0, pow(dT,3)*q2/2, 0,
    0, pow(dT,4)*q1/4, 0, pow(dT,3)*q1/2,
    pow(dT,3)*q2/2, 0, pow(dT,2)*q2, 0,
    0, pow(dT,3)*q1/2, 0, pow(dT,2)*q1;

    
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // Radar updates
      // Radar Matrices added to KF object
      ekf_.R_ = R_radar_;
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      
      // Radar Measurements added
      VectorXd z;
      z = VectorXd(3);
      z << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],measurement_pack.raw_measurements_[2];
      
      // Radar Update Called
      ekf_.UpdateEKF(z);
  } else {
    // Laser updates
      // Laser Matrices added to KF object
      ekf_.R_ = R_laser_;
      ekf_.H_ = H_laser_;
      
      // Laser Measurements added
      VectorXd z;
      z = VectorXd(2);
      z << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1];
      
      // Laser Measurements added
      ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
