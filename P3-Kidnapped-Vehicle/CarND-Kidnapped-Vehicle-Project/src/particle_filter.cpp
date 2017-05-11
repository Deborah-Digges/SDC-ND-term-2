/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	std::normal_distribution<double> N_x_init(0, std[0]);
	std::normal_distribution<double> N_y_init(0, std[1]);
	std::normal_distribution<double> N_theta_init(0, std[2]);
	std::default_random_engine gen;

	num_particles = 200;

	is_initialized = true;

	weights = std::vector<double>(num_particles);
	particles = std::vector<Particle>(num_particles);

	for(int i=0; i<num_particles; ++i) {
		weights.at(i) = 1;
		particles.at(i).weight = 1;
		particles.at(i).theta = theta + N_theta_init(gen);
		particles.at(i).x = x + N_x_init(gen);
		particles.at(i).y = y + N_y_init(gen);
		particles.at(i).id = i;

	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::normal_distribution<double> N_x_init(0, std_pos[0]);
	std::normal_distribution<double> N_y_init(0, std_pos[1]);
	std::normal_distribution<double> N_theta_init(0, std_pos[2]);
	std::default_random_engine gen;

	double x0 = 0;
	double y0 = 0;
	double theta0 = 0;

	double x_new = 0;
	double y_new = 0;
	double theta_new = 0;

	for(int i=0; i<num_particles; ++i) {
		x0 = particles.at(i).x;
		y0 = particles.at(i).y;
		theta0 = particles.at(i).theta;

		if(fabs(yaw_rate) < 1e-5) {
			x_new = x0 + velocity * delta_t * cos(theta0);
			y_new = y0 + velocity * delta_t * sin(theta0);
			theta_new = theta0;

		} else {
			x_new = x0 + (velocity/yaw_rate) * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
			y_new = y0 + (velocity/yaw_rate) * (cos(theta0) - cos(theta0 + yaw_rate * delta_t));
			theta_new = theta0 + yaw_rate * delta_t;
		}
		particles.at(i).x  = x_new + N_x_init(gen);
		particles.at(i).y = y_new + N_y_init(gen);
		particles.at(i).theta = theta_new + N_theta_init(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

Particle mapObservationToMapCoordinates(LandmarkObs observation, Particle particle) {
	double x = observation.x;
	double y = observation.y;

	double xt = particle.x;
	double yt = particle.y;
	double theta = particle.theta;

	Particle mapCoordinates;

	mapCoordinates.x = x * cos(theta) - y * sin(theta) + xt;
	mapCoordinates.y = x * sin(theta) + y * cos(theta) + yt;

	return mapCoordinates;

}

/**
 * Find the distance between a map landmark and a transformed particle
 */
double calculateDistance(Map::single_landmark_s land_mark, Particle map_coordinates) {
	double x1 = land_mark.x_f;
	double y1 = land_mark.y_f;

	double x2 = map_coordinates.x;
	double y2 = map_coordinates.y;

	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}


/**
 * Find the closest map landmark to a given transformed particle observation
 */
Map::single_landmark_s findClosestLandmark(Particle map_coordinates, Map map_landmarks) {
	Map::single_landmark_s closest_landmark = map_landmarks.landmark_list.at(0);
	double distance = calculateDistance(map_landmarks.landmark_list.at(0), map_coordinates);

	for(int i=1; i<map_landmarks.landmark_list.size(); ++i) {
		Map::single_landmark_s current_landmark = map_landmarks.landmark_list.at(i);
		double current_distance = calculateDistance(current_landmark, map_coordinates);

		if(current_distance < distance) {
			distance = current_distance;
			closest_landmark = current_landmark;
		}
	}
	return closest_landmark;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	for(Particle particle: particles) {
		for(LandmarkObs observation: observations) {
			// Convert the observation as seen by the particle into map coordinates
			Particle map_coordinates = mapObservationToMapCoordinates(observation, particle);
			Map::single_landmark_s closest_landmark = findClosestLandmark(map_coordinates, map_landmarks);
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
