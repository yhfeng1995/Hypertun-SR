#include "cost_evaluation.hpp"
#include "sparsestereo/sparsestereo-inl.h"
#include "sparsestereo/census-inl.h"
#include "sparsestereo/censuswindow.h"
#include "sparsestereo/imageconversion.h"
#include <iostream>
#include <cassert>
#include <bitset>

// cost_evaluation:
// inputs: 
// - I_l : image left [H x W]
// - I_r : image right [H x W]
// - D_it: Interpolated disparity [H x W]
// - G   : Matrix G which points to the corresponding triangle for every pixel [H x W]
// - O   : [N x (u,v)] matrix, containing all N pixels with high gradient
// - param: parameter struct
//
// outputs:
// - C_it  : Normalized cost associated to D_it
// - census_l: Census transformed left image
// - census_r:  Census transformed right image
// ############################################
// This function compares every patch with the correspondent patch, 
// given the interpolated disparity, using a census comparison.
// It returns a matirx containing the costs for every pixel. 
void cost_evaluation(cv::Mat &I_l, cv::Mat &I_r, 
                        cv::Mat &D_it, cv::Mat &G,
                         cv::Mat & O, parameters &param,
                         cv::Mat &C_it, cv::Mat &census_l, cv::Mat &census_r){



	// try with exfast census
	//####################################
	std::cout << "cost_evaluation.cpp census try" << std::endl;

	// define containers
	cv::Mat_<unsigned char> leftImg, rightImg;
	cv::Mat_<char> charLeft, charRight;
	cv::Mat_<unsigned int> censusLeft, censusRight;


	#pragma omp parallel sections default(shared) num_threads(2)
	{
		#pragma omp section
		{
			// Sign input image
			leftImg = I_l;

			// create container for char images
			charLeft = cv::Mat_<char>(param.H, param.W);

			// create container for census outputs
			censusLeft = cv::Mat_<unsigned int> (param.H, param.W);

			// convert images
			sparsestereo::ImageConversion::unsignedToSigned(leftImg, &charLeft);

			// perform census transform
			sparsestereo::Census::transform5x5(charLeft, &censusLeft);
		}

		#pragma omp section
		{
			// Sign input images
			rightImg = I_r;

			// create container for char images
			charRight = cv::Mat_<char>(param.H, param.W);

			// create container for census outputs
			censusRight = cv::Mat_<unsigned int> (param.H, param.W);

			// convert images
			sparsestereo::ImageConversion::unsignedToSigned(rightImg, &charRight);

			// perform census transform
			sparsestereo::Census::transform5x5(charRight, &censusRight);
		}
	}


	// loop over all high gradient pixels
	for (int it = 0; it < param.nOfHiGradPix; it++){

		// extract u,v
		int u = O.at<int>(it, 0);
		int v = O.at<int>(it, 1);

		// check if triangle is defined for this pixel
		if (G.at<int>(v,u) != -1){
			// get interpolated disparity
			int disp = D_it.at<float>(v,u);

			// be sure u+d does not exeed the image 
			if (u + disp > 1242){
				disp = disp - (u + disp - 1242);
			}

			sparsestereo::HammingDistance mHamming;
			// calculate hamming distance
			//std::cout << "caluclate hamming at (u+d,v) = (" << u << "+" << (int)disp << "," << v <<  ")" << std::endl;

			unsigned int currCensusLeft = censusLeft.at<unsigned int>(v,u);
			//std::cout << "extracted census Left:  " << std::bitset<24>(currCensusLeft) << std::endl;
			unsigned int currCensusRight = censusRight.at<unsigned int>(v,u + (int)disp);
			//std::cout << "extracted census Right: " << std::bitset<24>(currCensusRight) << std::endl;
			unsigned char hamming = mHamming.calculate(currCensusLeft, currCensusRight);


			// milan: mHamming.calculate returns sometimes a hamming = 25, Idk why...
			if (hamming >= 24){
				hamming = 24;
			}

			// normalize cost
			float n_cost = hamming / 24.0;
			assert((0 <= n_cost) && (n_cost <= 1));
			//std::cout << "n_cost: " << n_cost << std::endl;
			// write cost to C_it
			C_it.at<float>(v,u) = n_cost;
		}
	}

	// copy census transformed images for output
	census_l = censusLeft;
	census_r = censusRight;

}


// This helper takes a padded image and a pixel coordinate and the container for the census as argument
// It creates a binary descriptor for the neighborhood around the pixel, writes 1 if a pixel is smaller than the center.§§
void census (cv::Mat &paddedImg, int u_pad, int v_pad, std::bitset<24> &census){
	//cv::Mat census = cv::Mat::zeros(1, 24, CV_8U);

	// get intensity of center pixel
	uchar center = paddedImg.at<uchar>(v_pad, u_pad);

	int count = 0;

	std::bitset<24> mask(1);

	// loop over window
	for (int a = 0; a < 5; ++a){
		for (int b = 0; b < 5; ++b){
			// leave the center pixel out
			if  (!((a == 0) && (b == 0))){
				// get the value of the neighbor pixel
				uchar px = paddedImg.at<uchar>(v_pad - 2 + a, u_pad -2 + b);
				// set census to 1 if intenity is less
				if (px < center){
					census = census | (mask<<count);
					count++;
				} else{
					// else leave at zero
					count++;
				}
			}
		}
	}
}