#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "stats.hpp"
#include <stdio.h>
#include <iostream>


void pipeline_sgbm(cv::String filename_left, cv::String filename_right, cv::String filename_disp, stats &statistics);


#endif