#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ =  F * x + u; 
  P = F * P * F.transpose();
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateHelper(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = z(0);
  float py = z(1);
  float vx = z(2);
  float vy = z(3);
  float range = sqrt(px * px + py * py);
  float angle = atan2 (py, px);
  float rangeRate = (px * vx + py * vy) /  sqrt(px * px + py * py);
  while (angle < -M_PI || angle > M_PI) {
  	if (angle < -M_PI) {
  		angle += 2 * M_PI;
  	} else if (angle > M_PI) {
  		angle -= 2 * M_PI;
  	}
  }
  h << sqrt(px * px + py * py),  atan2 (py, px), (px * vx + py * vy) /  sqrt(px * px + py * py);
  VectorXd h(3);
  VectorXd y = z - h;
  UpdateHelper(y);
}

void KalmanFilter::UpdateHelper(const Vector &y) {
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
