#ifndef SPARSE_STEREO_HPP
#define SPARSE_STEREO_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <omp.h>

#include "../ThirdParty/sparsestereo/exception.h"
#include "../ThirdParty/sparsestereo/extendedfast.h"
#include "../ThirdParty/sparsestereo/stereorectification.h"
#include "../ThirdParty/sparsestereo/sparsestereo-inl.h"
#include "../ThirdParty/sparsestereo/census-inl.h"
#include "../ThirdParty/sparsestereo/imageconversion.h"
#include "../ThirdParty/sparsestereo/censuswindow.h"

namespace Hypertun_SR
{
    void sparse_stereo(cv::Mat I_l, cv::Mat I_r, cv::Mat &S);
}

#endif // SPARSE_STEREO_HPP
