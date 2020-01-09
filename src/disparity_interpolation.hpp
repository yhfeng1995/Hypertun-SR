#ifndef DISPARITY_INTERPOLATION_HPP
#define DISPARITY_INTERPOLATION_HPP

#include <opencv2/opencv.hpp>
#include "parameters.hpp"

void disparity_interpolation(cv::Mat &G, cv::Mat &T, cv::Mat &O, parameters & param,
                                 cv::Mat &D_it);

#endif // DISPARITY_INTERPOLATION_HPP