#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                const std::vector<Eigen::VectorXd>& ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate RMSE using new input.
  */
  Eigen::VectorXd CalculateRMSEContinuous(const Eigen::VectorXd& estimations,
                                          const Eigen::VectorXd& ground_truth,
                                          const Eigen::VectorXd& rmse,
                                          const int count);

};

#endif /* TOOLS_H_ */
