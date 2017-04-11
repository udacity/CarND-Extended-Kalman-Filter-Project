#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::Matrix4d P_;

  // state transistion matrix
  Eigen::Matrix4d F_;

  // process covariance matrix
  Eigen::Matrix4d Q_;

  // measurement matrix
  //Eigen::MatrixXd H_;
  Eigen::Matrix<double, 2, 4> H_laser_;
  Eigen::Matrix<double, 4, 2> Ht_laser_;
  Eigen::Matrix<double, 3, 4> Hj_;
  Eigen::Matrix<double, 4, 3> Htj_;

  // measurement covariance matrix
  //Eigen::MatrixXd R_;
  Eigen::Matrix2d R_laser_;
  Eigen::Matrix3d R_radar_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::Vector2d &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::Vector3d &z);

private:
  Eigen::Matrix4d I;
};

#endif /* KALMAN_FILTER_H_ */
