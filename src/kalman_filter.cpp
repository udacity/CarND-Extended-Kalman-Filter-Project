#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


/* 
 * Please note that the Eigen library does not initialize 
 * VectorXd or MatrixXd objects with zeros upon creation.
 *
 * Ref: https://en.wikipedia.org/wiki/Kalman_filter
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float rho = sqrt(px * px + py * py);

    // Handle divided by zero
    if (rho < 0.00001)
    {
        px += 0.001;
        py += 0.001;
        rho = sqrt(px * px + py * py);
    }

    float phi = atan2(py, px);
    float rho_dot = (px * vx + py * vy) / rho;

    VectorXd hx(3);
    hx << rho, phi, rho_dot;

    VectorXd y = z - hx;

    // Normalize angle to within -PI and PI if needed
    while (y(1) > M_PI || y(1) < -M_PI)
    {
        if (y(1) > M_PI) y(1) -= M_PI;
        else if (y(1) < -M_PI) y(1) += M_PI;
    }

    CommonUpdate(y);
}

/**
 * Common method to update KF with provided y value
 * @param y
 */
void KalmanFilter::CommonUpdate(const VectorXd &y)
{
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
