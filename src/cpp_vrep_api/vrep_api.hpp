#ifndef CPP_VREP_API_VREP_API_H_
#define CPP_VREP_API_VREP_API_H_

#include <iostream>
#include <cassert>  
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "lkas_model.hpp"
#include "paths.hpp"
#include "config_vrep.hpp"
#include "utils.hpp"

extern "C" {
    #include "extApi.h"
}

class vrepAPI : lkasModel,utils {
private:
    // private members
    int m_clientID, m_ping_time, m_cam, m_car, m_floor; /// client ID of the API connected to VREP
							/// handles to ping time, camera, car and floor
    int m_resolution[2]; /// resolution of the image
    simxUChar* m_image;  /// handle to capture image from the camera in VREP
    int m_nakedCar_steeringLeft, m_nakedCar_steeringRight; /// handles to steer the car
    int m_nakedCar_motorLeft, m_nakedCar_motorRight; /// handles to move the car
    simxFloat m_position[3]; /// position of the car
    double m_desired_wheel_rot_speed;  /// desired car wheel rotation speed
    // private methods

    /**  @brief C++ implementation for converting VREP camera image to Matrix. From RGB to BGR.
	 @return    the matrix format (of the image in BGR)
      */
    cv::Mat vrep_img_2_Mat();
public:
    /// constructor
    vrepAPI();
    /// destructor
    ~vrepAPI();
    // public methods
    void sim_delay(int time_step);
    cv::Mat sim_sense();
    void sim_actuate(long double steering_angle);
};

#endif
