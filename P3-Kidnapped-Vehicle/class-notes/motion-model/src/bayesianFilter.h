/*
 * bayesianFilter.h
 *
 *  Created on: Apr 24, 2017
 *      Author: ddigges
 */

#ifndef BAYESIANFILTER_H_
#define BAYESIANFILTER_H_

#include <vector>
#include <string>
#include <fstream>

#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"

class bayesianFilter {
public:
	//constructor:
	bayesianFilter();
	//deconstructor:
	virtual ~bayesianFilter();


	//main public member function, which estimate the beliefs:
	void process_measurement(const MeasurementPackage &measurements,
			                 const map &map_1d,
			                 help_functions &helpers);

	//member public: belief of state x:
	std::vector<float> bel_x ;

private:

/////private members:

	////////////////
    //Add members!//
    ////////////////
	bool is_initialized_;
	float control_std;
	std::vector<float> bel_x_init;

};

#endif /* BAYESIANFILTER_H_ */
