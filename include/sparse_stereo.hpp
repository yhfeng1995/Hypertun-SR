#ifndef SPARSE_STEREO_HPP
#define SPARSE_STEREO_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <omp.h>

#include "sparsestereo/exception.h"
#include "sparsestereo/extendedfast.h"
#include "sparsestereo/stereorectification.h"
#include "sparsestereo/sparsestereo-inl.h"
#include "sparsestereo/census-inl.h"
#include "sparsestereo/imageconversion.h"
#include "sparsestereo/censuswindow.h"

void sparse_stereo(cv::Mat I_l, cv::Mat I_r, cv::Mat &S);

#endif // SPARSE_STEREO_HPP
