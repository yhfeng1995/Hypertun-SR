// Adapted from:
// http://www.jayrambhia.com/blog/disparity-mpas
// https://github.com/jayrambhia/Vision/blob/master/OpenCV/C%2B%2B/disparity.cpp


#include "pipeline_sgbm.hpp"

using namespace cv;
using namespace std;

void computeAccuracy(cv::Mat D_f, cv::String filename_disp, stats &statistics);

void pipeline_sgbm(cv::String filename_left, cv::String filename_right, cv::String filename_disp, stats &statistics) {

	cv::Mat I_l = cv::imread(filename_left);
	cv::Mat I_r = cv::imread(filename_right);

	boost::posix_time::ptime algorithm_time_start = boost::posix_time::microsec_clock::local_time();

	Mat g1, g2;
	Mat disp, disp8;

	cvtColor(I_l, g1, CV_BGR2GRAY);
	cvtColor(I_r, g2, CV_BGR2GRAY);

	// Parameter values according to:
	// http://www.cvlibs.net/datasets/kitti/eval_stereo_flow_detail.php?benchmark=stereo&result=3ae300a3a3b3ed3e48a63ecb665dffcc127cf8ab
	StereoSGBM sgbm;
	sgbm.SADWindowSize = 3;
	sgbm.numberOfDisparities = 128;
	sgbm.preFilterCap = 63;
	sgbm.minDisparity = 0;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = 1;
	sgbm.P1 = sgbm.SADWindowSize*sgbm.SADWindowSize*4;
	sgbm.P2 = sgbm.SADWindowSize*sgbm.SADWindowSize*32;

	sgbm(g1, g2, disp);
	normalize(disp, disp8, 0, 256, CV_MINMAX, CV_8U);

	boost::posix_time::time_duration algorithm_time_elapsed = (boost::posix_time::microsec_clock::local_time() - algorithm_time_start);
	std::cout << std::setprecision(3);
	std::cout << "#################################################" << std::endl;
	std::cout << "ALGORITHM TOOK: " << algorithm_time_elapsed.total_microseconds()/1.0e6 << " seconds" << std::endl; 
	std::cout << "WITH A SPEED OF: " << 1.0e6/algorithm_time_elapsed.total_microseconds() << " Hz" << std::endl;
	std::cout << "#################################################" << std::endl;

	cv::imshow("Disparity", disp8);
	std::cout << disp.type() << std::endl;

	statistics.alg_time = algorithm_time_elapsed.total_microseconds()/1.0e3;
	statistics.alg_freq = 1.0e6/algorithm_time_elapsed.total_microseconds();


	if(statistics.acc_calc)computeAccuracy(disp, filename_disp, statistics);

}

void computeAccuracy(cv::Mat D_f, cv::String filename_disp, stats &statistics){

	if (filename_disp == " ") return;

	std::cout << "################################################" << std::endl;
	std::cout << "ACCURACY (comparing with ground-truth disparity)" << std::endl;
	//std::cout << "################################################" << std::endl;

	cv::Mat D_gt_png = cv::imread(filename_disp);

	int D_gt_png_width = D_f.cols;
	int D_gt_png_height = D_f.rows;

	float gt_value;

	// initialize some parameters needed for calculation
	float n_loops = 0;
	float counter_2 = 0;
	float counter_3 = 0;
	float counter_4 = 0;
	float counter_5 = 0;
	float threshold_2 = 2;
	float threshold_3 = 3;
	float threshold_4 = 4;
	float threshold_5 = 5;

	//normalize(D_gt_png, D_gt_png, 0, 256, CV_MINMAX, CV_8U);

	// comparison with ground-truth disparity and 4 different threshold
	for (int32_t v=0; v<D_gt_png_height; v++) {
		for (int32_t u=0; u<D_gt_png_width; u++) {

			gt_value = D_gt_png.at<uint16_t>(v,u,0);
			float disp = D_f.at<int>(v,u,0)/256/256/16;
			// Why the 16?: http://stackoverflow.com/questions/33308326/why-does-stereosgbm-give-negative-and-above-numberofdisparities-numbers

			if (disp != 0 && gt_value != 0) { // both ground-truth and estimate disparity valid
				n_loops = n_loops + 1;	
				// ground-truth disparity computed by dividing pixel value by 256 
				if (abs(disp-gt_value/256.0) < threshold_5)  
					counter_5 = counter_5 + 1;
					if (abs(disp-gt_value/256.0) < threshold_4)
						counter_4 = counter_4 + 1;
						if (abs(disp-gt_value/256.0) < threshold_3)
							counter_3 = counter_3 + 1;
							if (abs(disp-gt_value/256.0) < threshold_2)
								counter_2 = counter_2 + 1;
			}
		}
	}

	statistics.acc2 = counter_2/n_loops * 100;
	statistics.acc3 = counter_3/n_loops * 100;
	statistics.acc4 = counter_4/n_loops * 100;
	statistics.acc5 = counter_5/n_loops * 100;

	std::cout << "less than 2 pixels: " << statistics.acc2 << " %" << std::endl;
	std::cout << "less than 3 pixels: " << statistics.acc3 << " %" << std::endl;
	std::cout << "less than 4 pixels: " << statistics.acc4 << " %" << std::endl;
	std::cout << "less than 5 pixels: " << statistics.acc5 << " %" << std::endl;

}