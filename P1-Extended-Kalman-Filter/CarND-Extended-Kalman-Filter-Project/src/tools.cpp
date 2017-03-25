#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
		VectorXd rmse(4);
		rmse << 0,0,0,0;

		// check the validity of the following inputs:
		//  * the estimation vector size should not be zero
		//  * the estimation vector size should equal ground truth vector size
		if(estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
			std::cout << "Invalid estimation or ground truth data";
			return rmse;
		}

		//accumulate squared residuals
		for(int i=0; i < estimations.size(); ++i){
	        VectorXd residuals(4);
	        residuals = estimations[i] - ground_truth[i];
	        residuals = residuals.array() * residuals.array();
	        rmse += residuals ;
		}

		//calculate the mean
		rmse = rmse/estimations.size();

		//calculate the squared root
		rmse = rmse.array().sqrt();

		//return the result
		return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	Hj << 0,0,0,0,
			0,0,0,0,
			0,0,0,0;

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if(px == 0 && py == 0) {
		return Hj;
	}

	float px2py2 = pow(px, 2) + pow(py, 2);

	Hj(0, 0) = px/sqrt(px2py2);
	Hj(0, 1) = py/sqrt(px2py2);

	Hj(1, 0) = -py/px2py2;
	Hj(1, 1) = px/px2py2;

	Hj(2, 0) = py* (vx*py - vy*px)/pow(px2py2, 3/2);
	Hj(2, 1) = px * (vy*px - vx*py)/pow(px2py2, 3/2);
	Hj(2, 2) = px/sqrt(px2py2);
	Hj(2, 3) = py/sqrt(px2py2);

	return Hj;
}
