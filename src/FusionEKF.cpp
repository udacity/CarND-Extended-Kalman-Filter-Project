#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using std::vector;

FusionEKF::FusionEKF() :
  is_initialized_(false), previous_timestamp_(0), 
  noise_ax_(9.0), noise_ay_(9.0) // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
{}

FusionEKF::~FusionEKF() {}

void FusionEKF::initialize(const MeasurementPackage &measurement_pack) {
  // Initialize the state ekf_.x_ with the first measurement.
  std::cout << "EKF: " << endl;
  previous_timestamp_ = measurement_pack.timestamp_;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Convert radar from polar to cartesian coordinates and initialize state.
    auto rho = measurement_pack.raw_measurements_[0];
    auto phi = measurement_pack.raw_measurements_[1];
    auto rate = measurement_pack.raw_measurements_[2];

    ekf_.x_ << rho*cos(phi), rho*sin(phi), rate*cos(phi), rate*sin(phi);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    if (ekf_.x_[0] == 0.0) ekf_.x_[0] = MIN_VAL; // init (to 0.01), to
    if (ekf_.x_[1] == 0.0) ekf_.x_[1] = MIN_VAL; // overcome divide by zero
  }

  is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    initialize(measurement_pack);
    return;   // done initializing, no need to predict or update
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //0. compute the time elapsed between the current and previous measurements
  auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;

  //2. Set the process covariance matrix Q
  auto dt2 = dt*dt, dt3 = dt*dt*dt / 2, dt4 = dt*dt*dt*dt / 4;
  ekf_.Q_(0, 0) = dt4*noise_ax_;
  ekf_.Q_(1, 1) = dt4*noise_ay_;
  ekf_.Q_(2, 0) = ekf_.Q_(0, 2) = dt3*noise_ax_;
  ekf_.Q_(3, 1) = ekf_.Q_(1, 3) = dt3*noise_ay_;
  ekf_.Q_(2, 2) = dt2*noise_ax_;
  ekf_.Q_(3, 3) = dt2*noise_ay_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update, as per sensor type
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.Hj_ = CalculateJacobian(ekf_.x_);
    auto z = Vector3d{ measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2] };
    ekf_.UpdateEKF(z);
  } else {
    // Laser updates
    auto z = Vector2d{ measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1] };
    ekf_.Update(z);
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << endl;
  std::cout << "P_ = " << ekf_.P_ << endl;
}

Vector4d FusionEKF::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  Vector4d rmse{ 0.0, 0.0, 0.0, 0.0 };

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
    || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array(); //coefficient-wise multiplication
    rmse += residual;
  }

  rmse = rmse / (double)estimations.size();       //calculate the mean
  rmse = rmse.array().sqrt();                     //calculate the squared root

  return rmse;
}

Matrix<double, 3, 4> FusionEKF::CalculateJacobian(const VectorXd& x_state) {
  Matrix<double, 3, 4> Hj;

  //recover state parameters
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  auto c1 = px*px + py*py;
  auto c2 = sqrt(c1);
  auto c3 = (c1*c2);

  //check division by zero
  if (fabs(c1) < MIN_VAL*MIN_VAL) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj <<
    (px / c2),                (py / c2),                0.0,      0.0,
    -(py / c1),               (px / c1),                0.0,      0.0,
    py*(vx*py - vy*px) / c3,  px*(px*vy - py*vx) / c3,  px / c2,  py / c2;

  return Hj;
}
