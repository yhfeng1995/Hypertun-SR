#ifndef SUPPORT_RESAMPLING_HPP
#define SUPPORT_RESAMPLING_HPP

#include <opencv2/opencv.hpp>
#include <bitset>
#include <cassert>
#include "parameters.hpp"
#include "cost_evaluation.hpp"

void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, parameters &param,
                            cv::Mat &I_l, cv::Mat &I_r,
                            cv::Mat &census_l, cv::Mat &census_r);

void epipolar_search(cv::Mat &I_l_p, cv::Mat &I_r_p,
                        int u, int v, int & d, parameters &param);
#endif // SUPPORT_RESAMPLING_HPP