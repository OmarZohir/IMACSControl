#ifndef CONFIG_WEBOTS_H_
#define CONFIG_WEBOTS_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

#include "paths.hpp"

#include "webots_api.hpp"
#include "Halide.h"
#include "lateral_Control_WEBOTS.hpp"
#include "lane_detection_webots.hpp"
#include "image_signal_processing.hpp"
using namespace std;

// ------------ defs -------------//
#define TOP_SPEED 60 		//!< maximum speed
#define VEH_SPEED 30 		//!< cruising speed
#define DRAW_SLIDING_WINDOWS 1  //!< select whether to draw the sliding window while tracking
#define WEBOTS_CAM 1            //!< select when VREP camera frames are used
#define RE_DRAW_IMAGE 1        	//!< run re-draw image function
//#define DEBUG 1		//!< print debug statements to check intermediary outputs. Results in slow simulation sometimes
#define WEBOTS_YPIXEL_PER_METRE 112.72
// ------------ for all pipeline version except VER:8 ---------//
//#define WEBOTS_REF_TUNE 3.355 // for 50kmph (city_straight_night_nostreet.wbt)
//#define WEBOTS_REF_TUNE 3.255 // for 50kmph (city_straight_night_nostreet.wbt) // VER:3
//#define WEBOTS_REF_TUNE 3.615 // for 50kmph (city_straight_night_nostreet.wbt) // VER:0
//#define WEBOTS_REF_TUNE 3.555 // for 50kmph (city_straight_night.wbt, city_straight_dawn.wbt)
#define WEBOTS_REF_TUNE 3.615 // for 50kmph (city_straight.wbt, city_straight_dusk.wbt, city_straight_foggy.wbt)
//#define WEBOTS_REF_TUNE 15.615 // for 50kmph (city_straight_snow.wbt)
//#define WEBOTS_REF_TUNE 30.815 // for 50kmph (city_straight_lqg.wbt)
// ------------ for pipeline VER:8 ---------//
//#define WEBOTS_REF_TUNE 5.415 // for 50kmph (city_straight_night_nostreet.wbt)
//#define WEBOTS_REF_TUNE 3.005 // for 50 kmph (city_straight_night.wbt, city_straight_dawn.wbt, city_straight_dusk.wbt)
//#define WEBOTS_REF_TUNE 2.985 // for 50 kmph (city_straight.wbt)
#define WEBOTS_LEFT_THRESOLD 100
#define WEBOTS_RIGHT_THRESOLD 400

#define FINE_TUNE 0.010652587064676618
#define ROAD_WIDTH 3.25

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

