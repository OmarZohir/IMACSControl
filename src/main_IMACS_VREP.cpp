/*
 *  Copyright (c) 2020 sayandipde
 *  Eindhoven University of Technology
 *  Eindhoven, The Netherlands
 *
 *  Name            :   cpp_vrep_framework.cpp
 *
 *  Authors         :   Sayandip De (sayandip.de@tue.nl)
 *			Sajid Mohamed (s.mohamed@tue.nl)
 *
 *  Date            :   March 26, 2020
 *
 *  Function        :   main file to run the imacs approx-ibc framework
 *
 *  History         :
 *      26-03-20    :   Initial version.
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */



#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath> 
#include "vrep_api.hpp"
#include "config_vrep.hpp"

int main(int argc, char **argv) {
	using namespace std;
	using namespace cv;
	if (argc < 2) {
		cout << "Minimum one argument needed.\n Usage: ./imacs_vrep {unsigned_int scenario} {float simstep}\n";
		cout << "Eg: ./imacs_vrep 1\t (default): ./imacs_vrep 1 0.005\n";
		return -1;
	}
	// ------- simulation parameters ----------//	
	vector<float> period_s {0.040, 0.025, 0.035, 0.040, 0.020, 0.030, 0.020, 0.020, 0.040}; //!< sampling period h in seconds for each scenario s_i
	vector<float> tau_s {0.0379, 0.0206, 0.0303, 0.0373, 0.0164, 0.0299, 0.0198, 0.0160, 0.0359}; //!< sensor-to-actuator delay \tau in seconds for each scenario s_i
	
	int scenario = atoi(argv[1]); //!< the system scenario to simulate
	int approximate_pipeline_version = 0; //!< the approximate pipeline version to simulate. See IEEE Access paper. 0: no approximation
	cout << "scenario: s" << scenario << "\tapproximate pipeline version: v"<< approximate_pipeline_version << endl;

	float simstep = 0.005; //!< V-REP logical simulation step time in seconds
	int simulation_time = 15; //!< Total logical simulation time in seconds
	float wait_time = 0.015;//2.5; //!< Initial wait time in seconds. For actual simulation, this should be set such that the required velocity is reached by the car. Typically, 2.5 seconds.
	if (argc == 3) {
		simstep = stof(argv[2]);
		cout << "simstep: "<< simstep << endl;
	}

	// simulation main loop parameters
	Mat img_vrep, img_isp; //!< image obtained from vrep, image after image signal processing (ISP) stage
	long double steering_angle =0.0f; //!< Computed steering angle
	vector<long double> steering_angles(2); //!< left and right steering angles
	float time = 0.0f; //!< variable to keep track of current simulation time
	float curr_period = 0.0f; //!< current sampling period
	long double yL = 0.0L; //!< yL: lateral deviation
	int it_counter = 0; //!< iteration counter
	int time_step; //!< to update current simulation time

	// ------- init simulation ----------//
	vrepAPI VrepAPI; //!< initialise vrepAPI
    	imageSignalProcessing ISP; //!< initialise ISP
    	laneDetection Lane_detection; //!< initialise lane detection
    	lateralControllerVREP Controller; //!< initialise lateral controller

	// --- delay 2.5 sec to reach desired velocity ---//		
	time_step = wait_time / simstep; //!< The controller is designed for a constant speed. The set vehicle speed should be reached first in VREP.
	cout << "\t\t\tinitialising... waiting for " << time_step << " simulation steps (to reach required vehicle speed)\n\t\t\t";
	VrepAPI.sim_delay(time_step);
	curr_period = period_s[scenario];
	cout << "\n\t\t\tCompleted initialisation" << endl;

	// ------- simulation main loop -------//
	cout << "control period :" << curr_period << " sec" << endl;
	cout << "control delay :" << tau_s[scenario] << " sec" << endl;
	cout << "simulation step :" << simstep << " sec" << endl;
	
	while(time < simulation_time - curr_period) { //!< simulate until the set simulation time
		cout << "\nsimulation_time: " << time << "\t";
		// ------- sensing ----------//	
		img_vrep = VrepAPI.sim_sense(); //!< image captured from VREP camera
	
		// ------- ISP: imageSignalProcessing ----------//
		img_isp = ISP.approximate_pipeline(img_vrep, approximate_pipeline_version); //!< 
			
		// ------- laneDetection -----------------//
		yL = Lane_detection.lane_detection_pipeline(img_isp);
		//cout << "lateral deviation: " << yL << endl;
		cout << "lateral deviation: " << yL;
		
		// ------- control_compute --------------//
		Controller.compute_steering_angle(yL, it_counter, scenario);
		steering_angle = Controller.get_steering_angle();

		// ------- control_next_state  -----------//
		Controller.estimate_next_state(it_counter, scenario);
			
		// ------- sensor-to-actuator delay ----------//
		time_step = ceil(tau_s[scenario] / simstep);
			
		VrepAPI.sim_delay(time_step);  
		time += time_step*simstep;
		// ----- actuating -----//
		// [TODO] add actuation delay here (too small, neglected)
		VrepAPI.sim_actuate(steering_angle); 

		// ------- remaining delay to sampling period ----------//
		time_step = floor((period_s[scenario] - tau_s[scenario])/ simstep);
		VrepAPI.sim_delay(time_step);  
		time += time_step*simstep;

		// ------- handle simulation time -------//
		//time += period_s[scenario];
		
		it_counter++;
	}
	
	cout << "images: " << it_counter << endl; 

	return 0;
}
