#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // State vector.
  Eigen::VectorXd x_;

  // State covariance matrix.
  Eigen::MatrixXd P_;

  // State transistion matrix.
  Eigen::MatrixXd F_;

  // Process covariance matrix.
  Eigen::MatrixXd Q_;

  // Measurement matrix.
  Eigen::MatrixXd H_;

  // Measurement covariance matrix.
  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Predicts the state and the state covariance using the process model.
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state using standard Kalman Filter equations.
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state using Extended Kalman Filter equations.
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
