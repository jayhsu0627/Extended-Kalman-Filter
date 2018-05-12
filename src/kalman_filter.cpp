#include "kalman_filter.h"
#include <iostream>
using namespace std;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // object state
  P_ = P_in; // object covariance matrix
  F_ = F_in; // state transition matrix
  H_ = H_in; // measurement matrix
  R_ = R_in; // measurement covariance matrix
  Q_ = Q_in; // process covariance matrix
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "UpdateEKF X_: " << x_ << endl;
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1) , x_(0));
  while ((theta - z(1) )>pi/2)
    theta = theta - pi ;
  while ((z(1) - theta)>pi/2)
    theta = theta + pi ;
  double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  VectorXd h = VectorXd(3); // h(x_)
  h << rho, theta, rho_dot;
  VectorXd y = z - h;

	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);

	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}