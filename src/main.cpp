#include <iostream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "google/cloud/pubsub/publisher.h"
#include "google/cloud/pubsub/subscriber.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace pubsub = google::cloud::pubsub;

int main(int argc, char* argv[]) try {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " <project-id> <subscription-id>\n";
    return 1;
  }

  std::string const project_id = argv[1];
  std::string const subscription_id = argv[2];

  auto receive_messages = [](pubsub::Subscriber subscriber) {
    std::mutex mu;
    std::condition_variable cv;
    int message_count = 0;
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    FusionEKF fusionEKF;
    Tools tools;

    auto session = subscriber.Subscribe(
        [&](pubsub::Message const& m, pubsub::AckHandler h) {

          std::cout << "\nRECEIVED: " << m.data();

          string a(m.data());
          istringstream iss(a);

          string sensor_type;
          iss >> sensor_type;
          long timestamp;

          MeasurementPackage meas_package;
          GroundTruthPackage gt_package;

          if (sensor_type.compare("L") == 0) {
            // LASER MEASUREMENT

            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

          } else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT

            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          // Read ground truth data.
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          gt_package.gt_values_ = VectorXd(4);
          gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

          // Predict and update.
          fusionEKF.ProcessMeasurement(meas_package);

          cout << "PREDICTION: ";
          cout << fusionEKF.ekf_.x_(0) << "\t";
          cout << fusionEKF.ekf_.x_(1) << "\t";
          cout << fusionEKF.ekf_.x_(2) << "\t";
          cout << fusionEKF.ekf_.x_(3) << "\t\n";

          rmse = tools.CalculateRMSEContinuous(fusionEKF.ekf_.x_,
                                               gt_package.gt_values_,
                                               rmse,
                                               message_count);

          cout << "ACCURACY - RMSE: ";
          cout << rmse(0) << "\t";
          cout << rmse(1) << "\t";
          cout << rmse(2) << "\t";
          cout << rmse(3) << "\t\n";

          std::unique_lock<std::mutex> lk(mu);
          ++message_count;
          lk.unlock();
          cv.notify_one();
          // Ack the message.
          std::move(h).ack();
          cout << "# ACK'ED: " << message_count << "\n";
        });

    std::unique_lock<std::mutex> lk(mu);
    cv.wait_for(lk,
                30 * 1000ms,
                [&message_count] { return message_count > 10000; });
    lk.unlock();
    // Cancel the subscription session.
    session.cancel();
    // Wait for the session to complete, no more callbacks after this point.
    auto status = session.get();
    // Report any final status, blocking.
    std::cout << "Message count: " << message_count << ", status: " << status
              << "\n";
  };

  receive_messages(
      pubsub::Subscriber(
          pubsub::MakeSubscriberConnection(
              pubsub::Subscription(project_id, subscription_id))));

} catch (std::exception const& ex) {
  std::cerr << "Standard exception raised: " << ex.what() << "\n";
  return 1;
}
