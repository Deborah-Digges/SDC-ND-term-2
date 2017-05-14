#include "PID.h"
#include <iostream>
#include <ctime>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
	p_error = Kp * cte;
	std::cout << "Kp, cte,p_error: "<< Kp << ", "<< cte << ", " << p_error << "\n";
	d_error = Kd * 1000000 *(cte - this->prev_cte)/(CLOCKS_PER_SEC);
	std::cout << "cte, prev_cte, dt,d_errror: " << cte << "," << this->prev_cte << this->dt/CLOCKS_PER_SEC << ", " << d_error << "\n";
	//i_error = Ki * cte;
}

double PID::TotalError() {
	return p_error + d_error + i_error;
}
