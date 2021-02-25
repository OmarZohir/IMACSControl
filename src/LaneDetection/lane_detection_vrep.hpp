#ifndef LANEDETECTION_LANE_DETECTION_H_
#define LANEDETECTION_LANE_DETECTION_H_

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Halide.h"
#include "polyfit.hpp"
#include "config_vrep.hpp"
#include "paths.hpp"

class laneDetection : pathsIMACS {
private:
    std::vector<long double> m_yL_container;
	std::vector<long double> m_ref_container;
    // private methods
    void bev_transform(cv::Mat& src, cv::Mat& dst);
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst);
    std::vector<std::vector<cv::Point2f>> get_bev_points();
    std::vector<std::vector<cv::Point>> sliding_window_lane_tracking(cv::Mat& src);
    std::vector<long double> calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds, 
                                            std::vector<cv::Point> right_lane_inds);
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi, 
                            cv::Mat& img_warped, cv::Mat& img_lanes_temp, cv::Mat& img_detected_lanes, 
                            cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped);
    //string lane_out_img_dir = "/home/student/cppVrepLKAS/imacs/out_imgs/";
public:
    // constructor
    laneDetection();
    // destructor
    ~laneDetection();
    // class public methods
    long double lane_detection_pipeline(cv::Mat src);
    std::vector<long double> get_yL_container();
	std::vector<long double> get_ref_container();
};

#endif
