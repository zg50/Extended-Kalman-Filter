#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0) {
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    return rmse;
  }
 
  for (unsigned i=0; i < estimations.size(); ++i) {
    VectorXd temp = estimations[i] - ground_truth[i];
    temp = temp.array() * temp.array();
    rmse += temp;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  if (x_state.size() != 4) {
	std::cout << "the size of x_state in CalculateJacobian must be 4";
    return Hj;
  }
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE
  float a = px * px + py * py;
  float b = px * (px * vy - py * vx);
  float c = py * (py * vx - px * vy);
  // check division by zero
  if (a == 0) {
    std::cout << "division by zero" << std::endl;
    return Hj;
  }
  // compute the Jacobian matrix
  Hj << px / sqrt(a), py / sqrt(a), 0, 0,
        -py / a, px / a, 0, 0, 
         c / (a * sqrt(a)), b / (a * sqrt(a)), px / sqrt(a), py / sqrt(a);
  return Hj;
}
