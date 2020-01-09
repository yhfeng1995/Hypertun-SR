#include "support_resampling.hpp"
#include "sparsestereo/sparsestereo-inl.h"
#include "sparsestereo/census-inl.h"
#include "sparsestereo/censuswindow.h"
#include "sparsestereo/imageconversion.h"
#include <iostream>

void showMatch (cv::Mat &I_l, cv::Mat &I_r, cv::Mat &S_epi, int epiL);

// support_resampling :
// inputs:
// - C_g   : Cost associated with regions of high confidence matches [H' x W' x (u, v, d, cost)]
// - C_b   : Cost associated with refions of invalid disparities [H' x W' x (u, v, cost)]
// - S_it  : Sparse support pixels with valid depths [N x (u, v, d)]
// - param : Parameter struct
// - I_l   : Image left
// - I_r   : Image right
// - census_l : census transformed left image
// - census_r : census transformed right image
//
// outputs:
// - S_it  : Updated sparse support pixels with valid depths
// #############################################################################
// This function loops over C_g and C_b. The pixels and the correspondendt depths
// in C_g are written as new support points to S_it.
// The pixels from C_b are collected in X and passed to the epipolar search. This 
// returns the matching pixels of the right image and the correspondendt depths.
// Those new found matches are then also added to S_it.
void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, parameters &param,
							cv::Mat &I_l, cv::Mat &I_r,
							cv::Mat &census_l, cv::Mat &census_r){

	std::cout << "support_resampling.cpp" << std::endl;


	// get number of points in C_b and C_g
	int noBadPts = 0;
	int noGoodPts = 0;
	for (int it_u=0; it_u<param.W_bar; ++it_u){
		for (int it_v=0; it_v<param.H_bar; ++it_v){
			// if cost is higher than t_hi
			if ((param.t_hi < C_b.at<float>(it_v, it_u, 2)) && (C_b.at<float>(it_v, it_u, 2) <= 1)){
				noBadPts++;
			}
			// if cost is lower than t_lo	
			if (C_g.at<float>(it_v, it_u, 3) < param.t_lo){
				noGoodPts++;
			}	
		}
	}


	// container for bad points
	int sz_X[] = {noBadPts, 2};
	cv::Mat X = cv::Mat(2, sz_X, CV_32F, cv::Scalar::all(0));
	int X_length = 0;

	// container for good points
	int sz_add[] = {noGoodPts, 3};
	cv::Mat S_add = cv::Mat(2, sz_add, CV_32F,cv::Scalar::all(0));
	int S_add_length = 0;

	// loop over C_b and C_g, store the data to X or S_add if costs are >t_hi or <T_Lo respectively
	for (int v_bar = 0; v_bar < param.H_bar; ++v_bar){
		for (int u_bar = 0; u_bar < param.W_bar; ++u_bar){
			// if C_b != empty at (v_bar, u_bar)
			if ((param.t_hi < C_b.at<float>(v_bar, u_bar, 2)) && (C_b.at<float>(v_bar, u_bar, 2) <= 1)){
				// be sure that u and v are in range
				assert(0 <= C_b.at<float>(v_bar, u_bar, 0) && C_b.at<float>(v_bar, u_bar, 0) <= 1242);
				assert(0 <= C_b.at<float>(v_bar, u_bar, 1) && C_b.at<float>(v_bar, u_bar, 1) <= 375);
				// be sure that costs are in range
				assert(0 <= C_b.at<float>(v_bar, u_bar, 2) && C_b.at<float>(v_bar, u_bar, 2) <= 1);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 3) && C_g.at<float>(v_bar, u_bar, 3) <= 1);

				// store (u,v) for bad point for resampling
				X.at<float>(X_length, 0) = C_b.at<float>(v_bar, u_bar, 0);
				X.at<float>(X_length, 1) = C_b.at<float>(v_bar, u_bar, 1);

				X_length++;
			}
			// check if cost is low enough but not zero
			if (C_g.at<float>(v_bar, u_bar, 3) < param.t_lo){
				// be sure that u and v are in range
				assert(0 <= C_g.at<float>(v_bar, u_bar, 0) && C_g.at<float>(v_bar, u_bar, 0) <= 1242);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 1) && C_g.at<float>(v_bar, u_bar, 1) <= 375);
				// be sure that costs are in range
				assert(0 <= C_b.at<float>(v_bar, u_bar, 2) && C_b.at<float>(v_bar, u_bar, 2) <= 1);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 3) && C_g.at<float>(v_bar, u_bar, 3) <= 1);

				// store (u,v,d) for valid points
				S_add.at<float>(S_add_length, 0) = C_g.at<float>(v_bar, u_bar, 0);
				S_add.at<float>(S_add_length, 1) = C_g.at<float>(v_bar, u_bar, 1);
				S_add.at<float>(S_add_length, 2) = C_g.at<float>(v_bar, u_bar, 2);

				S_add_length++;
			}
		}
	}

	// define container for epipolar search [noBadPts x (u, v, d)]
	cv::Mat S_epi;
	S_epi = cv::Mat(noBadPts, 3, CV_32F, 0.0);

	sparsestereo::HammingDistance mHamming;

	int epiLength = 0;

	int pattern_sz = 7;

	//loop over bad points
	for (int i=0; i < noBadPts; ++i){

		// assume nonzero disparity
		int currU = X.at<float>(i, 0);
		int currV = X.at<float>(i, 1);

		// be sure that u and v are in range
		assert((0 <= currU) && ( currU <= 1242));
		assert((0 <= currV) && ( currV <= 375));

		// check if pixels in pattern are within image
		if ((currV - pattern_sz >= 0) && (currV + pattern_sz < param.H)
			 && (currU - pattern_sz >= 0) && (currU + pattern_sz < param.W)){
			
			// get current left census pattern of four pixel: center, Up, Down, Left, Right
			unsigned int currCensusLeft = census_l.at<unsigned int>(currV, currU);
			unsigned int currCensusLeftU = census_l.at<unsigned int>(currV - pattern_sz, currU);
			unsigned int currCensusLeftD = census_l.at<unsigned int>(currV + pattern_sz, currU);
			unsigned int currCensusLeftL = census_l.at<unsigned int>(currV, currU - pattern_sz);
			unsigned int currCensusLeftR = census_l.at<unsigned int>(currV, currU + pattern_sz);

			// define best cost
			float bestCost = 6.;
			int u_best = 0;
			int nuOfZeros = 0;

			// window on epipolar line
			int win = param.epi_window;

			// loop along each epipolar line
			for (int u_ = std::max(pattern_sz, currU - win); u_ < std::min(currU, param.W - pattern_sz); ++u_){

				//extract each right census pattern
				unsigned int currCensusRight = census_r.at<unsigned int>(currV, u_);
				unsigned int currCensusRightU = census_r.at<unsigned int>(currV - pattern_sz, u_);
				unsigned int currCensusRightD = census_r.at<unsigned int>(currV + pattern_sz, u_);
				unsigned int currCensusRightL = census_r.at<unsigned int>(currV, u_ - pattern_sz);
				unsigned int currCensusRightR = census_r.at<unsigned int>(currV, u_ + pattern_sz);

				// calculate Hamming distances
				unsigned char hamming = mHamming.calculate(currCensusLeft, currCensusRight);
				unsigned char hammingU = mHamming.calculate(currCensusLeftU, currCensusRightU);
				unsigned char hammingD = mHamming.calculate(currCensusLeftD, currCensusRightD);
				unsigned char hammingL = mHamming.calculate(currCensusLeftL, currCensusRightL);
				unsigned char hammingR = mHamming.calculate(currCensusLeftR, currCensusRightR);

				float currCost = hamming / 24.0f;
				float currCostU = hammingU / 24.0f;
				float currCostD = hammingD / 24.0f;
				float currCostL = hammingL / 24.0f;
				float currCostR = hammingR / 24.0f;

				float totCost = currCost + currCostU + currCostD + currCostL + currCostR; 

				

				// store best match
				if (totCost < bestCost){
					bestCost = totCost;
					u_best = u_;
				}

				// count how many zero-cost matches there are
				if (totCost == 0)
					nuOfZeros++;

			}


			// check if best match is good and unique
			if (bestCost < param.t_epi){   // (bestCost == 0) && (nuOfZeros == 1)
				// calculate disparity with u_left - u_right
				int disp = currU - u_best;
				assert(disp >= 0);

				// save (u, v, d) to epi
				S_epi.at<float>(epiLength, 0) = X.at<float>(i, 0);
				S_epi.at<float>(epiLength, 1) = X.at<float>(i, 1);
				S_epi.at<float>(epiLength, 2) = float(disp);

				epiLength++;
			}
		}

		

	}


	// combine S_it, S_add and S_epi
	cv::Mat S_next;
	S_next = cv::Mat(S_it.rows + noGoodPts + epiLength, 3, CV_32F);

	for (int i = 0; i < S_it.rows + noGoodPts + epiLength; ++i){
		if (i < S_it.rows){
			S_next.at<float>(i, 0) = S_it.at<float>(i, 0);
			S_next.at<float>(i, 1) = S_it.at<float>(i, 1);
			S_next.at<float>(i, 2) = S_it.at<float>(i, 2);
		} else if(i < S_it.rows + noGoodPts){
			S_next.at<float>(i, 0) = S_add.at<float>(i - S_it.rows, 0);
			S_next.at<float>(i, 1) = S_add.at<float>(i - S_it.rows, 1);
			S_next.at<float>(i, 2) = S_add.at<float>(i - S_it.rows, 2);
		} else{
			S_next.at<float>(i, 0) = S_epi.at<float>(i - S_it.rows - noGoodPts, 0);
			S_next.at<float>(i, 1) = S_epi.at<float>(i - S_it.rows - noGoodPts, 1);
			S_next.at<float>(i, 2) = S_epi.at<float>(i - S_it.rows - noGoodPts, 2);
		}
	}
	S_it = S_next; 


	std::cout << X_length << " points stored for epi-search." << std::endl;
	std::cout << S_add_length << " points stored directly as support points." << std::endl;
	std::cout << epiLength << " points stored from the epi-search." << std::endl;

}

// Inputs:
// - I_l_p	  : padded image left
// - I_r_p    : padded image right
// - u and v  : pixel coordinates of point of interest
// - params   : parameter struct
//
// Outputs:
// - d        : disparity of best match   
// ############################################################################################    
// This sub-routine takes the padded left and right image, a pixelcoordinate u/v,
// a container for the disparity d and the parameter struct.
// It searches along the epipolar line (horizontal) and calculates for every pixel in the
// right image the census transform and the hemming distance to the census transform of the pixel
// u/v in the left image. The output is the disparity to the pixel with the smallest hemming cost.
void epipolar_search(cv::Mat &I_l_p, cv::Mat &I_r_p,
                        int u, int v, int & d, parameters &param){
	// be sure that u and v are in range
	assert(0 <= u && u <= 1242);
	assert(0 <= v && v <= 375);

	// get padded indices
	int u_p = u + 2;
	int v_p = v + 2;
	// be sure that u_p and v_p are in range
	assert(0 <= u_p && u_p <= 1242);
	assert(0 <= v_p && v_p <= 375);

	// get census of keypoint
	std::bitset<24> cens_l(0);
	assert(v_p > 0);
	census(I_l_p, u_p, v_p, cens_l);

	// set best cost to high and best indices to zero
	double best_cost = 1.5;
	int u_best = 0;

	// loop along the epipolar line
	for (int u_ = 0; u_ <= u_p - 2; ++u_){

		// get census of current pixel
		std::bitset<24> cens_c(0);
		census(I_r_p, u_ + 2, v_p, cens_c);

		// calculate error
		ushort cost = 0;
		std::bitset<24> mask(1);
		std::bitset<24> result(0);

		// calculating Hemming distance
		result = cens_l ^ cens_c;
		for (int it = 0; it < 24; ++it){
			std::bitset<24> current_bit = (result & (mask<<it))>>it;
			if (current_bit == mask){
				cost++;
			}
		}

		// normalize cost
		float n_cost = cost / 24.0;

		// if error < best_error : keep v_
		if (n_cost < best_cost){
			best_cost = n_cost;
			u_best = u_;
		}
	}

	// calculate disparity with u_left - u_right
	d = u - u_best;
}



void showMatch (cv::Mat &I_l, cv::Mat &I_r, cv::Mat &S_epi, int epiL){
	cv::Mat im1 = I_l.clone();
	cv::Mat im2 = I_r.clone();

	cv::Size sz1 = im1.size();
	cv::Size sz2 = im2.size();

	cv::Mat im3(sz1.height, sz1.width + sz2.width, CV_8UC1);
	cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
	im1.copyTo(left);

	cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
	im2.copyTo(right);

	cv::cvtColor(im3, im3, CV_GRAY2RGB);

	cv::RNG rng(424242);
	for (int i = 0; i < epiL; ++i){
		cv::Vec3b color;
		color = cv::Vec3b(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::Point ptL, ptR;
		ptL.x = S_epi.at<float>(i, 0);
		ptL.y = S_epi.at<float>(i, 1);
		int d = S_epi.at<float>(i, 2);

		ptR.x = ptL.x + sz1.width - d;
		ptR.y = ptL.y;
		cv::circle(im3, ptL, 5, (cv::Scalar) color, 2);
		cv::circle(im3, ptR, 5, (cv::Scalar) color, 2);
		cv::line(im3, ptL, ptR, (cv::Scalar) color);
	}

	std::ostringstream oss;
	oss << "matches " << rng.uniform(0, 15);
	std::string str = oss.str();

	cv::imshow(str, im3);
	//cv::waitKey(0);

}