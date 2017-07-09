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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
   H_laser_ << 1, 0, 0, 0,
   0, 1, 0, 0;
   
   Hj_ << 1, 1, 0, 0,
   1, 1, 0, 0,
   1, 1, 1, 1; 
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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    
    
    ekf_.F_ = MatrixXd(4, 4);
    
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ << 1, 0,
    0, 1;
    
    ekf_.Q_ = MatrixXd(4, 4);
    
    float px;
    float py;
    float vx;
    float vy;
    
    ekf_.H_ = MatrixXd(2,4);
    ekf_.H_ << 1, 0, 0, 0,
    0, 1, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_d = measurement_pack.raw_measurements_[2];
      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_d * cos(phi);
      vy = rho_d * sin(phi);
       
      ekf_.x_ << px, py, vx, vy;
      
      //Hj_ = tools.CalculateJacobian(ekf_.x_); // Revisar
      
      std::cout << "Inicio RADAR " <<  ekf_.x_ << std::endl;
      
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      vx = 0.0;
      vy = 0.0;
      
      ekf_.x_ << px, py, vx, vy; 
      
      std::cout << "Inicio LASER " <<  ekf_.x_ << std::endl;     
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
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
  std::cout << "Calcular Tiempo" << std::endl; 
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  std::cout << "Iniciar Matriz F" << std::endl;
  ekf_.F_ << 1, 0, dt, 0,
  0, 1, 0, dt,
  0, 0, 1, 0,
  0, 0, 0, 1;
  std::cout << "Calcular Q" << std::endl;
  float noise_ax = 9;
  float noise_ay = 9;
  
  ekf_.Q_ << 	((pow(dt,4)/4)*noise_ax), 0, ((pow(dt,3)/2)*noise_ax), 0,
				0, ((pow(dt,4)/4)*noise_ay), 0, ((pow(dt,3)/2)*noise_ay),
				((pow(dt,3)/2)*noise_ax), 0, pow(dt, 2)*noise_ax, 0,
				0, ((pow(dt,3)/2)*noise_ay), 0, pow(dt, 2)*noise_ay; 
				
  std::cout << "Prediccion Q_ " <<  ekf_.Q_ << std::endl;
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
    
      VectorXd z_Radar(3); 
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_d = measurement_pack.raw_measurements_[2];
 
      z_Radar << rho, phi , rho_d;
      
      ekf_.UpdateEKF(z_Radar);
      
  } else {
    // Laser updates
      VectorXd z_Laser(2);
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
	  
	  z_Laser << px, py;
	  
	  ekf_.Update(z_Laser);
  
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
