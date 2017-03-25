#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd h(VectorXd);
double SNormalizeAngle(double phi);

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
	std::cout << "X_:" <<  x_ << std::endl;
	x_ = F_ * x_;
	std::cout << "New X_:" <<  x_ << std::endl;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.rows();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

/*
 * This takes a 4d vector [px, py, vx, vy] which represents
 * the predicted state and returns the projection
 * into the measurement space [rho, phi, rho dot]
 */
VectorXd h(VectorXd predicted_state) {
	double px = predicted_state[0];
	double py = predicted_state[1];
	double vx = predicted_state[2];
	double vy = predicted_state[3];

	VectorXd projected_state = VectorXd(3);
	projected_state << 0,0,0;

	if(px == 0 && py == 0) {
		return projected_state;
	}

	double rho = sqrt(pow(px, 2) + pow(py, 2));
	double phi = atan(py/px);
	double rho_dot = ((px * vx) + (py * vy))/rho;

	projected_state[0] = rho;
	projected_state[1] = phi;
	projected_state[2] = rho_dot;

	return projected_state;

}

double SNormalizeAngle(double phi)
{
  const double Max = M_PI;
  const double Min = -M_PI;

  return phi < Min
    ? Max + std::fmod(phi - Min, Max - Min)
    : std::fmod(phi - Min, Max - Min) + Min;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
		TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	// Use the non-linear function to project the predicted state
	// into the measurement space
	std::cout << "z:" << z << std::endl;
	VectorXd z_pred = h(x_);
	std::cout << "zpred:" << z_pred << std::endl;
	VectorXd y = z - z_pred;

	y[1] = SNormalizeAngle(y[1]);
	std::cout << "y:" << y << std::endl;
	MatrixXd Ht = H_.transpose();
	std::cout << "Ht:" << Ht << std::endl;
	MatrixXd S = H_ * P_ * Ht + R_;
	std::cout << "S" << S << std::endl;
	MatrixXd Si = S.inverse();
	std::cout << "S_inv:" << Si << std::endl;
	MatrixXd PHt = P_ * Ht;
	std::cout << "PHt:" << PHt << std::endl;
	MatrixXd K = PHt * Si;
	std::cout << "K:" << K << std::endl;
	//new estimate
	x_ = x_ + (K * y);
	std::cout << "x_:" << x_ << std::endl;
	long x_size = x_.rows();
	std::cout << "x_size" << x_size << std::endl;
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


