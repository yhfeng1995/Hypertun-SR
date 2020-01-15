#ifndef IMAGE_GRADIENT_HPP
#define IMAGE_GRADIENT_HPP

#include <opencv2/core/core.hpp>
#include "parameters.hpp"

namespace Hypertun_SR
{
    void image_gradient(cv::Mat &image_in, parameters &param, cv::Mat &O);
}  // namespace Hypertun_SR


#endif //IMAGE_GRADIENT_HPP