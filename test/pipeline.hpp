#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include "stats.hpp"

void pipeline(cv::String filename_left, cv::String filename_right, cv::String filename_disp, stats &statistics);



#endif