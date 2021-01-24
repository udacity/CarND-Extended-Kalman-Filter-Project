#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

/**
 * This is an const utility to organize sensor types and measurements
 *
 */
class MeasurementPackage
{
public:
    enum SensorType
    {
        LASER,
        RADAR
    } sensor_type_;

    long long timestamp_;

    Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
