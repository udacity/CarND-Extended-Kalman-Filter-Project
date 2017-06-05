#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  // measurement timestamp in microsecs
  long long timestamp_us_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // Measurement vector indexes
  enum LASER_MES_E{
      LASER_X = 0,
      LASER_Y,

      LASER_MES_SIZE
  };
  enum RADAR_MES_E{
      RADAR_RO = 0,
      RADAR_THETA,
      RADAR_RO_DOT,

      RADAR_MES_SIZE
  };

  Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
