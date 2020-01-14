#include "disparity_interpolation.hpp"
#include <cassert>
#include <iostream>


// inputs:
// - G   : HxW Matrix with an index for each pixel to the corresponding triangle
// - T   : 4 x num_riangles matrix with the plane parameters for each pixel
// - O   : [N x (u,v)] matrix, containing all N pixels with high gradient
//
// outputs:
// - D_it: HxW matrix with an interpolated disparity for each pixel (sparse!)
void disparity_interpolation(cv::Mat &G, cv::Mat &T, cv::Mat &O, parameters & param, cv::Mat &D_it){
	
#ifdef HPTSR_DEBUG
	std::cout << "disparity_interpolation.cpp" << std::endl;
#endif

	// get image dimensions
	int H = G.rows;
	int W = G.cols;

	// loop over all high gradient pixels
	for (int it = 0; it < param.nOfHiGradPix; it++){

		// extract u,v
		int u = O.at<int>(it, 0); // j
		int v = O.at<int>(it, 1); // i		

		// get index of the corresponding triangle
		int idx_trig = G.at<int>(v,u);

		// check if triangle exists
		if(idx_trig >= 0){
			// get plane parameters
			float a = T.at<float>(0,idx_trig);
			float b = T.at<float>(1,idx_trig);
			float c = T.at<float>(2,idx_trig);
			float d = T.at<float>(3,idx_trig);

			// interpolate disparity
			float disp = (d - a*u - b*v) / c;
			D_it.at<float>(v,u) = (disp>0) ? disp : 0;			
		}
	}
	// std::cerr << "Disparity_interpolation fail points: " << fail_cnt << std::endl;
	// double D_it_min, D_it_max;
	// cv::minMaxIdx(D_it, &D_it_min, &D_it_max);
	// std::cout << "D_it min value: " << D_it_min << std::endl;
	// std::cout << "D_it max_value: " << D_it_max << std::endl;
}