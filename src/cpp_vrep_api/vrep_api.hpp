#ifndef CPP_VREP_API_VREP_API_H_
#define CPP_VREP_API_VREP_API_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "lkas_model.hpp"

extern "C" {
    #include "extApi.h"
}

class vrepAPI : lkasModel{
private:
    // private members
    int m_clientID, m_ping_time, m_cam, m_car, m_floor;
    int m_resolution[2];
    simxUChar* m_image;
    int m_nakedCar_steeringLeft, m_nakedCar_steeringRight;
    int m_nakedCar_motorLeft, m_nakedCar_motorRight;
    simxFloat m_position[3];
    double m_desired_wheel_rot_speed;  
    // private methods
    cv::Mat vrep_img_2_Mat();
public:
    // constructor
    vrepAPI();
    // destructor
    ~vrepAPI();
    // public methods
    void sim_delay(int time_step);
    cv::Mat sim_sense();
    void sim_actuate(long double steering_angle);
};

#endif
