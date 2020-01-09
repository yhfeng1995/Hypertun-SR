#ifndef COST_EVALUATION_HPP
#define COST_EVALUATION_HPP

#include <opencv2/opencv.hpp>
#include <bitset>
#include <omp.h>
#include "parameters.hpp"

void cost_evaluation(cv::Mat &I_l, cv::Mat &I_r, 
                        cv::Mat &D_it, cv::Mat &G,
                         cv::Mat & O, parameters &param,
                         cv::Mat &C_it, cv::Mat &census_l, cv::Mat &census_r);

void census(cv::Mat &paddedImg, int i_pad, int j_pad, std::bitset<24> &census);

#endif // COST_EVALUATION_HPP