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
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

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

	/**
	 TODO:

	 Complete the initialization. See ukf.h for other member properties.

	 Hint: one or more values initialized above might be wildly off...
	 */
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
		// Initialize state x based on whether RADAR or LIDAR
		// timestamp
		// return
	// Calculate delta_t
	// Call Predict
	// Check Sensor Type
		// RADAR
			// Set R
			// UpdateRadar
		// Laser
			// Set R
			// Update Laser
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**
	 TODO:

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
	 TODO:

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
	 TODO:

	 Complete this function! Use radar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the radar NIS.
	 */
	// Map the predicted state x_ to the the measurement space
		// Use the predicted sigma points
		// Transform each predicted sigma point to measurement space
		// Calculate the mean z_pred and the covariance S of the predicted points

	// Calculate the cross-correlation matrix
	// Use these to update the state x_ and P_ using the Kalman Gain
	// Calculate NIS
}
