#ifndef CONFIG_VREP_H_
#define CONFIG_VREP_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
//!< Header files to include after defining paths
#include "Halide.h"
#include "lateral_Control_VREP.hpp"
#include "lane_detection_vrep.hpp"
#include "image_signal_processing.hpp"
#include "paths.hpp"

using namespace std;

// ------------ defs -------------//
    #define VREP_CAM 1               // select when VREP camera frames are used
    #define SELECT_PERIOD 1
    #define DRAW_SLIDING_WINDOWS 1   // select whether to draw the sliding window while tracking
    #define RE_DRAW_IMAGE 1          // run re-draw image function

// ------------ function declarations -------------//

template<class Container>
std::ostream& write_containers(	const Container& c1,
								const Container& c2,
								const Container& c3,
								std::ostream& out,
								char delimiter = ',')
{
	out << "lateral_deviation(m)";
	out << delimiter;
    bool write_sep = false;
    for (const auto& e: c1) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(left)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c2) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(right)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c3) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
    return out;
}

template<class Container>
std::ostream& write_containers_wref(	const Container& c1,
								const Container& c4,
								const Container& c2,
								const Container& c3,
								std::ostream& out,
								char delimiter = ',')
{
	out << "lateral_deviation(m)";
	out << delimiter;
    bool write_sep = false;
    for (const auto& e: c1) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "lane_reference(m)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c4) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(left)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c2) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
	out << "\n";
	out << "steering_angle(right)";
	out << delimiter;
    write_sep = false;
    for (const auto& e: c3) {
        if (write_sep)
            out << delimiter;
        else
            write_sep = true;
        out << e;
    }
	
    return out;
}

// void write_yL_2_file(std::vector<long double> yL_container, int pipeline_version);
std::string get_timestamp();



#endif

