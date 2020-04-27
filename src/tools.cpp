#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
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
  VectorXd sumResid(4);
  sumResid << 0, 0, 0, 0;
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  int n = estimations.size();
  if (n > 0 && n == ground_truth.size()) {
  // TODO: accumulate squared residuals
    for (int i = 0; i < n; ++i) {
        // ... your code here
        VectorXd error = estimations[i] - ground_truth[i];
        sumResid = sumResid.array() + error.array()*error.array();
    }
  }
  // TODO: calculate the mean
  // TODO: calculate the square-root
  rmse = (sumResid.array()/n).sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE
  float den = px*px + py*py;

  // check division by zero
  if (den > 0) {
  // compute the Jacobian matrix
      float sqrtDen = sqrt(den);
      float sqrtDen3 = den*sqrtDen;
      Hj << px/sqrtDen, py/sqrtDen, 0,  0,
            -py/den,    px/den,     0,  0,
            (vx*py - vy*px)*py/sqrtDen3, (vy*px - vx*py)*px/sqrtDen3,\
            Hj(0, 0),   Hj(0, 1);
  } else {
      cout << "CalculateJacobian(): Error -- division by zero." << endl;
  }
  return Hj;
}
