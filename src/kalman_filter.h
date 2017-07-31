#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transistion matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
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
    * Prediction Predicts the state and the state covariance
    * using the process model
    * @param delta_T Time between k and k+1 in s
    */
    void Predict();

    /**
    * Updates the state by using standard Kalman Filter equations
    * @param z The measurement at k+1
    */
    void Update(const Eigen::VectorXd &z);

    /**
    * Updates the state by using Extended Kalman Filter equations
    * @param z The measurement at k+1
    */
    void UpdateEKF(const Eigen::VectorXd &z);

private:
    Tools tools_;
};

#endif /* KALMAN_FILTER_H_ */
