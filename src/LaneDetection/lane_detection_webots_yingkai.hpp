#ifndef LANEDETECTION_LANE_DETECTION_WEBOTS_H_
#define LANEDETECTION_LANE_DETECTION_WEBOTS_H_

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Halide.h"
#include "polyfit.hpp"
#include "config.hpp"


#define WEBOTS_YPIXEL_PER_METRE 112.72 // 111.82//112.72(default)
#define WEBOTS_REF_TUNE -0.55          //3.133 3.633
#define WEBOTS_LEFT_THRESOLD 100
#define WEBOTS_RIGHT_THRESOLD 400

class laneDetectionWEBOTS : pathsIMACS {
private:
   std::vector<long double> m_yL_container;
        std::vector<long double> m_ref_container;
    // private methods
    long double x1, x2, x3, x4, y1, y2, y3,y4, x1d,x2d, x3d, x4d, y1d, y2d,y3d,y4d;
    void bev_transform(cv::Mat& src, cv::Mat& dst, int kfturn, int lane);
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst, int kfturn, int lane);
    std::vector<std::vector<cv::Point2f>> get_bev_points(int kfturn, int lane);
    std::vector<std::vector<cv::Point>> sliding_window_lane_tracking(cv::Mat& src);
    long double calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds,
                                            std::vector<cv::Point> right_lane_inds, int kfturn, int lane, int light, int imgcount);
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi,
                            cv::Mat& img_warped, cv::Mat& img_lanes_temp, cv::Mat& img_detected_lanes,
                            cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped, int kfturn, int lane);
public:
    // constructor
    laneDetectionWEBOTS();
    // destructor
    ~laneDetectionWEBOTS();
    long double lane_detection_pipeline(cv::Mat src, int kfturn,int lane, int light, int imgcount);
    std::vector<long double> get_yL_container();
    std::vector<long double> get_ref_container();
};

#endif
