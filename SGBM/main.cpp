#include "pipeline_sgbm.hpp"
#include "stats.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main() {

	const int ACCURACY_DATASET = 0;
	const int PERFORMANCE_DATASET = 1;

	int DATASET = ACCURACY_DATASET;

	cv::String path_left;
	cv::String path_right;
	cv::String path_disp;

	//setting folder paths and choosing all .png files
	if (DATASET == ACCURACY_DATASET) {
		path_left = "../data/data_stereo_flow/training/colored_0/*.png";
		path_right = "../data/data_stereo_flow/training/colored_1/*.png";
		path_disp = "../data/data_stereo_flow/training/disp_occ/*.png";
	}
	if (DATASET == PERFORMANCE_DATASET) {
		path_left = "../data/data_odometry_color/00/image_2/*.png";
		path_right = "../data/data_odometry_color/00/image_3/*.png";
		path_disp = "../data/data_odometry_color/00/image_3/*.png";
	}
	
	//adressing filenames with indexes
	std::vector<cv::String> filenames_left;
	std::vector<cv::String> filenames_right;
	std::vector<cv::String> filenames_disp;
	cv::glob(path_left, filenames_left);
	cv::glob(path_right, filenames_right);
	cv::glob(path_disp, filenames_disp);
	int num_files = filenames_left.size();


	const int ALL = 0;
	const int ONE = 1;
	bool single_image = false;

	
	//#######################
	int range = 15;               //<<<<-------------- // set here range to 'ALL', 'ONE' or a specific number
	//#######################
	if (DATASET == PERFORMANCE_DATASET) range = ALL;

	// prepare range of images
	if (range == ALL){
		// loop over all images
		range = num_files;
	} else
	if (range == ONE){
		// just do one image
		range = 1;
		single_image = true;
	}



	// create instance of stats
	stats statistics;
	if(DATASET == ACCURACY_DATASET) statistics.acc_calc = true;
	else statistics.acc_calc = false;

	// container for average numbers
	float avrg_PL_time = 0;
	float avrg_PL_freq = 0;
	float avrg_ALG_time = 0;
	float avrg_ALG_freq = 0;

	float avrg_acc2 = 0;
	float avrg_acc3 = 0;
	float avrg_acc4 = 0;
	float avrg_acc5 = 0;


	for (size_t i=0; i<range; i++) {
		if (single_image) i = 9;
		
		boost::posix_time::ptime time_start = boost::posix_time::microsec_clock::local_time();

		if (DATASET == PERFORMANCE_DATASET) pipeline_sgbm(filenames_left[i*2], filenames_right[i*2], " ", statistics);
		else pipeline_sgbm(filenames_left[i*2], filenames_right[i*2], filenames_disp[i], statistics);

		boost::posix_time::time_duration time_elapsed = (boost::posix_time::microsec_clock::local_time() - time_start);

		avrg_PL_time += time_elapsed.total_microseconds()/1.0e3;
		avrg_PL_freq += 1.0e6/time_elapsed.total_microseconds();
		avrg_ALG_time += statistics.alg_time;
		avrg_ALG_freq += statistics.alg_freq;
		if (statistics.acc_calc){
			avrg_acc2 += statistics.acc2;
			avrg_acc3 += statistics.acc3;
			avrg_acc4 += statistics.acc4;
			avrg_acc5 += statistics.acc5;
		} 

		std::cout << "#################################################" << std::endl;
		std::cout << std::setprecision(3);
		std::cout << std::setw(50) << std::left << "PIPELINE TOOK: " << std::right << time_elapsed.total_microseconds()/1.0e3 << " ms" << std::endl; 
		std::cout << std::setw(50) << std::left << "WITH A SPEED OF: " << std::right << 1.0e6/time_elapsed.total_microseconds() << " Hz" << std::endl;
		std::cout << "#################################################" << std::endl;

		if (DATASET == ACCURACY_DATASET) cv::waitKey(0);
		if (DATASET == PERFORMANCE_DATASET) cv::waitKey(1);
	}

	if (range != 1){
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::setw(50) << std::left << "ANALYSIS OVER IMAGES:" << std::right << range << std::endl;
		std::cout << std::endl;
		std::cout << std::setprecision(3);
		std::cout << std::setw(50) << std::left << "PIPELINE TOOK ON AVERAGE: " << std::right << avrg_PL_time / range << " ms" << std::endl;
		std::cout << std::setw(50) << std::left << "WITH AN AVERAGE SPEED OF: " << std::right << avrg_PL_freq / range << " Hz" << std::endl;
		std::cout << std::endl;
		std::cout << std::setw(50) << std::left << "ALGORITHM TOOK ON AVERAGE: " << std::right << avrg_ALG_time / range << " ms" << std::endl;
		std::cout << std::setw(50) << std::left << "WITH AN AVERAGE SPEED OF: " << std::right << avrg_ALG_freq / range << " Hz" << std::endl;
		std::cout << std::endl;
		std::cout << std::setw(50) << std::left << "AVERAGE ACCURACY " << std::endl;
		std::cout << std::setw(50) << std::left << "less than 2 pixels deviation: " << std::right << avrg_acc2 / range << " %" << std::endl;
		std::cout << std::setw(50) << std::left << "less than 3 pixels deviation: " << std::right << avrg_acc3 / range << " %" << std::endl;
		std::cout << std::setw(50) << std::left << "less than 4 pixels deviation: " << std::right << avrg_acc4 / range << " %" << std::endl;
		std::cout << std::setw(50) << std::left << "less than 5 pixels deviation: " << std::right << avrg_acc5 / range << " %" << std::endl;
		std::cout << "#################################################" << std::endl;
		std::cout << "#################################################" << std::endl;

	}
	return 0;

}