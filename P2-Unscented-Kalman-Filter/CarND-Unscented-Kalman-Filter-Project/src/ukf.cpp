#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
	//	 TODO: Change values for std_a_ and std_yawdd_

	// Initially the UKF is not initialized
	is_initialized_ = false;

	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);
	x_.fill(0.0);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);
	P_.fill(0.0);

	// Number of rows in our state vector
	n_x_ = x_.rows();

	// Number of rows in our state vector + 2 rows for the noise processes
	n_aug_ = n_x_ + 2;

	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 30;

	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;

	lambda_ = 3 - n_aug_;

	weights_ = VectorXd(2*n_aug_ + 1);

	weights_(0) = lambda_ / float(lambda_ + n_aug_);
	double common_weight = 1 / float(2 *(lambda_ + n_aug_));

	for(int i=1; i<weights_.size(); ++i) {
	  weights_(i) = common_weight;
	}

	NIS_radar_ = 0;
	NIS_laser_ = 0;

	R_laser_ = MatrixXd(2, 2);
	R_laser_ << pow(std_laspx_, 2), 0,
				0, pow(std_laspy_, 2);

	R_radar_ = MatrixXd(3, 3);
	R_radar_ << pow(std_radr_, 2), 0, 0,
				0, pow(std_radphi_, 2), 0,
				0, 0, pow(std_radrd_, 2);

//	std::cout << "is_initialized_" << is_initialized_ << std::endl;
//	std::cout << "use_laser_" << use_laser_ << std::endl;
//	std::cout << "use_radar_" << use_radar_ << std::endl;
//	std::cout << "x_" << x_ << std::endl;
//	std::cout << "P_" << P_ << std::endl;
//	std::cout << "Xsig_pred_" << Xsig_pred_ << std::endl;
//	std::cout << "std_a_" << std_a_ << std::endl;
//	std::cout << "std_yawdd_" << std_yawdd_ << std::endl;
//	std::cout << "std_laspx_" <<  std_laspx_ << std::endl;
//	std::cout << "std_laspy_" << std_laspy_ << std::endl;
//	std::cout << "std_radr_" <<std_radr_ << std::endl;
//	std::cout << "std_radphi_" << std_radphi_ << std::endl;
//	std::cout << "std_radrd_" << std_radrd_  << std::endl;
//	std::cout << "weights_" << weights_ << std::endl;
//	std::cout << "n_x_" << n_x_ << std::endl;
//	std::cout << "n_aug_" << n_aug_ << std::endl;
//	std::cout << "lambda_" << lambda_ << std::endl;
//	std::cout << "NIS_radar_" << NIS_radar_ << std::endl;
//	std::cout << "NIS_laser_" << NIS_laser_ << std::endl;
//	std::cout << "R_laser_" << R_laser_ << std::endl;
//	std::cout << "R_radar_" << R_radar_ << std::endl;
}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Make sure you switch between lidar and radar
	 measurements.
	 */
	// If not initialized, initialize
	if(!is_initialized_) {
		// Initialize state x based on whether RADAR or LIDAR
		if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			double rho_dot = meas_package.raw_measurements_[2];
			x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
		} else if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
		}
		previous_timestamp_ = meas_package.timestamp_;
		return;
	}

	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;

	if(dt > 0.001) {
		Prediction(dt);
	}

	if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
	} else {
		UpdateLidar(meas_package);
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**
	 Complete this function! Estimate the object's location. Modify the state
	 vector, x_. Predict sigma points, the state, and the state covariance matrix.
	 */
	// Create Augmented state mean vector x_aug and augmented state covariance matrix P_aug
	// Generate Sigma points for the augmented state matrix(7, 2*7 +1)
	// Use the prediction function to predict the k+1 values for these sigma points matrix(5, 2*7+1)
		// TODO: Need to store these predicted sigma points for use in the measurement update step
	// Use these values to compute the mean and covariance for the state predicted at time k+1
	// Store the state in x_ and P_
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	 Complete this function! Use lidar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the lidar NIS.
	 */
	// Calculate Z-pred using the H matrix
		// We have the predicted state x_ (5 x 1) which we want to transform to the measurement space(2 X 1)
		// We need a 2 x 5 H matrix:
		// [ 1 0 0 0 0]
	    // [ 0 1 0 0 0]
		// Calculate the z value corresponding to the predicted state: z = H * x_
		// R is the 2 x 2 Matrix - TODO: need to know the noise of laser for px and py measurements
		// TODO: Add R_laser as a member of the ukf class
	// Use Kalman Filter equations to update x_ and P_
	// Calculate NIS
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	 Complete this function! Use radar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the radar NIS.
	 */
	// Map the predicted state x_ to the the measurement space
		// Use the predicted sigma points
		// Transform each predicted sigma point to measurement space
		// Calculate the mean z_pred and the covariance S of the predicted points
			// TODO: For this need the covariance matrix R_RADAR [3 x 3] to be added

	// Calculate the cross-correlation matrix
	// Use these to update the state x_ and P_ using the Kalman Gain
	// Calculate NIS
}
