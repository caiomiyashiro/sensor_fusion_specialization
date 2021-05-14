#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if(px == 0 && py == 0){
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;
  }
  
  // compute the Jacobian matrix
  float pxy2 = pow(px, 2) + pow(py, 2);
  float c1 = vx*py - vy*px;
  float c2 = vy*px - vx*py;

  Hj << px/sqrt(pxy2),        py/sqrt(pxy2),        0,              0,
        -py/pxy2,             -px/pxy2,             0,              0,
        py*c1/pow(pxy2, 3/2), px*c2/pow(pxy2, 3/2), px/sqrt(pxy2),  py/sqrt(pxy2);

  return Hj;
}