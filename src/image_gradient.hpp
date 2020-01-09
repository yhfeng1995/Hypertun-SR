#ifndef IMAGE_GRADIENT_HPP
#define IMAGE_GRADIENT_HPP

#include <opencv2/core/core.hpp>
#include "parameters.hpp"

void image_gradient(cv::Mat &image_in, cv::Mat &image_out, parameters &param);

#endif //IMAGE_GRADIENT_HPP