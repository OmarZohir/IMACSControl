#ifndef LANEDETECTION_LANE_DETECTION_WEBOTS_H_
#define LANEDETECTION_LANE_DETECTION_WEBOTS_H_

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Halide.h"
#include "polyfit.hpp"
#include "config_webots.hpp"

/// @brief Class for lane detection WEBOTS. Inherits the base class pathsIMACS to load the paths to image.
class laneDetectionWEBOTS : pathsIMACS {
private:
    float x1, y1, x2, y2, x3, y3, x4, y4; 		/// RoI selection: (xi,yi) are the four co-ordinates of the RoI
    float x1d, y1d, x2d, y2d, x3d, y3d, x4d, y4d;	/// Bird's eye view (BEV) destination co-ordinates for BEV transformation
    std::vector<long double> m_yL_container;		/// container to store the yL values
    //std::vector<long double> m_ref_container;		/// container to store the reference values

    void bev_transform(cv::Mat& src, cv::Mat& dst, int world_encode);
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst, int world_encode);
    std::vector<std::vector<cv::Point2f>> get_bev_points(int world_encode);
    std::vector<std::vector<cv::Point>> sliding_window_lane_tracking(cv::Mat& src);
    long double calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds,
                                        std::vector<cv::Point> right_lane_inds);
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi,
                        cv::Mat& img_warped, cv::Mat& img_lanes_temp, cv::Mat& img_detected_lanes,
                        cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped, int world_encode);
    //string lane_out_img_dir = "/home/student/cppVrepLKAS/imacs/out_imgs/";
public:
    // constructor
    laneDetectionWEBOTS();
    // destructor
    ~laneDetectionWEBOTS();
    // class public methods
    long double lane_detection_pipeline(cv::Mat src, int world_encode, int pipeline_version);
    std::vector<long double> get_yL_container();
    //std::vector<long double> get_ref_container();
};

#endif
