#ifndef DISPARITY_REFINEMENT_HPP
#define DISPARITY_REFINEMENT_HPP

#include <opencv2/opencv.hpp>
#include <cassert>
#include "parameters.hpp"

void disparity_refinement(cv::Mat &D_it, cv::Mat &C_it,
                             cv::Mat &G, cv::Mat & O,
                             cv::Mat &D_f, cv::Mat &C_f, 
                             cv::Mat &C_g, cv::Mat &C_b,
                             parameters &param);

#endif // DISPARITY_REFINEMENT_HPP