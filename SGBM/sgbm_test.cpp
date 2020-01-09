// Adapted from:
// http://www.jayrambhia.com/blog/disparity-mpas
// https://github.com/jayrambhia/Vision/blob/master/OpenCV/C%2B%2B/disparity.cpp

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char const *argv[]) {


	Mat img1, img2, g1, g2;
	Mat disp, disp8;

	img1 = imread("../data/data_scene_flow/testing/image_2/000000_10.png");
	img2 = imread("../data/data_scene_flow/testing/image_3/000000_10.png");
	

	cvtColor(img1, g1, CV_BGR2GRAY);
	cvtColor(img2, g2, CV_BGR2GRAY);

	StereoSGBM sgbm;
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -0;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;

	sgbm(g1, g2, disp);
	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	imshow("left", img1);
	imshow("right", img2);
	imshow("disp", disp8);
	waitKey(0);

	return 0;
}


