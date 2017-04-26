#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
 public:
  int64_t timestamp_;

  enum SensorType {
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd gt_values_;
};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
