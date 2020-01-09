

#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>
#include <cassert>

#include "sparse_stereo.hpp"
#include "delaunay_triangulation.hpp"
#include "disparity_interpolation.hpp"
#include "cost_evaluation.hpp"
#include "disparity_refinement.hpp"
#include "support_resampling.hpp"
#include "parameters.hpp"
#include "image_gradient.hpp"
#include "stats.hpp"


// header of 'line2'
void line2(cv::Mat& img, const cv::Point& start, const cv::Point& end, 
                     const cv::Scalar& c1,   const cv::Scalar& c2);
// header of 'showGrid'
void showGrid(cv::Mat I_l, cv::Mat S, cv::Mat E, std::string str);

// header of 'showG'
void showG (cv::Mat I_l, cv::Mat G, parameters param, std::string str);

// header of 'showDisparity'
void showDisparity(cv::Mat I_l, cv::Mat D_it, std::string str);


// header of 'showSupportPts'
void showSupportPts(cv::Mat I_l, cv::Mat S_it, std::string str);

// header of ´computeAccuracy'
void computeAccuracy(cv::Mat D_f, cv::String filename_disp, stats &statistics);


void pipeline(cv::String filename_left, cv::String filename_right, cv::String filename_disp, stats &statistics) {
	std::cout << "#######" << std::endl;
	std::cout << "DATASET: http://www.cvlibs.net/datasets/kitti/eval_stereo_flow.php?benchmark=flow" << std::endl;
	std::cout << "#######" << std::endl << std::endl;
	std::cout << "Using CV version: " << CV_VERSION << std::endl;
	std::cout << "pipeline.cpp" << std::endl;
	std::cout << std::setprecision(3);

	//Load parameters
	parameters param;
	param.sz_occ = 32;
	param.n_iters = 2;
	param.t_lo = 2.f/24; // placeholder, verify optimal value
	param.t_hi = 21.f/24; // placeholder, verify optimal value
	param.im_grad = 20;
	param.t_epi = 0.3f;
	param.epi_window = 80;

	// Load images with time estimation
	boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::local_time();
	cv::Mat I_l = cv::imread(filename_left, CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat I_r = cv::imread(filename_right, CV_LOAD_IMAGE_GRAYSCALE);
	boost::posix_time::time_duration elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for loading the current images: " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
	
	// Start timer for performance of whole algorithm
	boost::posix_time::ptime algorithm_time_start = boost::posix_time::microsec_clock::local_time();

	// estimate time of image preprocessing
	lastTime = boost::posix_time::microsec_clock::local_time();

	// crop image to be dividable by 16
	int offset_u = 5;
	int offset_v = 2;
	cv::Rect roi; // region of interest
	roi.x = offset_u;
	roi.y = offset_v;
	roi.width = 1216;
	roi.height = 368;

	cv::Mat I_l_c = I_l(roi);
	cv::Mat I_r_c = I_r(roi);
	cv::Mat I_l_cb;

	// apply gaussian blur
	cv::GaussianBlur(I_l_c, I_l_cb, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);


	// Generate gradient image
	cv::Mat grad_l;

	// Replacced with OpenCV Sobel implementation 
	image_gradient(I_l_cb, grad_l, param);
	// cv::Mat grad_l_x, grad_l_y, grad_l_x2, grad_l_y2;
	// cv::Sobel(I_l_cb, grad_l_x, CV_32F, 1, 0);
	// cv::Sobel(I_l_cb, grad_l_y, CV_32F, 0, 1);
	// cv::pow(grad_l_x, 2, grad_l_x2);
	// cv::pow(grad_l_y, 2, grad_l_y2);
	// cv::sqrt(grad_l_x2+grad_l_y2, grad_l);
	// cv::convertScaleAbs(grad_l, grad_l);

	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for image preprocessing: " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;

	// gather high gradient pixels in Mat O with time calculation
	lastTime = boost::posix_time::microsec_clock::local_time();

	cv::Mat O = cv::Mat(0, 2, CV_32S);
	int highGradCount = 0;
	for (int vv = 0; vv < I_l_cb.rows; vv++){
		for (int uu = 0; uu < I_l_cb.cols; uu++){
			if(grad_l.at<uchar>(vv, uu) > param.im_grad){
				cv::Mat pixel = cv::Mat(1, 2, CV_32S);
				pixel.at<int>(0,0) = uu;
				pixel.at<int>(0,1) = vv;
				O.push_back(pixel);

				highGradCount++;
			}
		}
	}

	// for (int vv = 0; vv < I_l_cb.rows; vv++){
	// 	const uchar *p_grad = grad_l.ptr<uchar>(vv);
	// 	for (int uu = 0; uu < I_l_cb.cols; uu++){
	// 		if(p_grad[uu] > param.im_grad){
	// 			cv::Mat pixel = (cv::Mat_<int>(1,2) << uu, vv);
	// 			O.push_back(pixel);

	// 			highGradCount++;
	// 		}
	// 	}
	// }

	param.nOfHiGradPix = highGradCount;
	

	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for building the Mat O: " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;

	
	std::cout << "Number of pixels with high gradient: " << param.nOfHiGradPix << std::endl;


	// Get image height and width
	param.H = I_l_c.rows;
	param.W = I_r_c.cols;

	// Get initial grid height and width
	param.H_bar = std::floor(param.H / param.sz_occ);
	param.W_bar = std::floor(param.W / param.sz_occ);

	// Initialize final disparity and associated cost
	cv::Mat D_f = cv::Mat(param.H, param.W, CV_32F, 0.0);
	cv::Mat C_dummy_2 = cv::Mat(param.H, param.W, CV_32F, 0.0); // dummy array
	cv::Mat C_f = cv::Mat(param.H, param.W, CV_32F, param.t_hi);

	// Declare other variables
	cv::Mat S; // set of N support points with valid depths, Nx3 with [u,v,d]
	cv::Mat G; // graph: corresponding triangle of each pixel from delaunay triangulation
	cv::Mat T; // Triangle 4D plane parameters from delaunay triangulation
	cv::Mat E; // Triangle edges for plotting
	cv::Mat D; // dense piece-wise planar disparity
	cv::Mat C; // cost associated to D
	cv::Mat C_g; // cost associated with regions of good matches
	cv::Mat C_b; // cost associated with regions of bad matches 
	cv::Mat D_it; // Intermediate disparity (interpolated)
	cv::Mat C_it; // Cost associated to D_it
	cv::Mat census_l; // census transformed left image
	cv::Mat census_r; // census transformed right image

	G = cv::Mat(param.H, param.W, CV_32S, cv::Scalar(-1));

	// execute 'sparse_stereo' with elapsed time estimation 
	lastTime = boost::posix_time::microsec_clock::local_time();
	sparse_stereo(I_l_c, I_r_c, S);
	int num_S_points_init = S.rows;
	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for 'sparse_stereo': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;

	// execute 'delaunay_triangulation' with elapsed time estimation
	lastTime = boost::posix_time::microsec_clock::local_time();
	delaunay_triangulation(S, G, T, E);
	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for 'delaunay_triangulation': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;

	// show matrix G
	//showG(I_l, G, param, "G after delaunay");
	
	// set all support points in G to -1
	for (int j=0; j<S.rows; ++j){
		int u = S.at<float>(j,0);
		int v = S.at<float>(j,1);
		G.at<int>(v,u) = -1;
	}
	
	// show the grid from the delaunay triangulation
	//showGrid(I_l_c, S, E, "Initial Delaunay");

	for (int i = 0; i < param.n_iters; ++i) {
		std::cout << "################################################" << std::endl;
		std::cout << "ITERATION :: " << i+1 << std::endl;
		std::cout << "################################################" << std::endl;

		// initialize D_it and C_it new for every iteration
		D_it = cv::Mat(param.H, param.W, CV_32F, 0.0);
		C_it = cv::Mat(param.H, param.W, CV_32F, param.t_hi);

		// execute 'disparity_interpolation' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		disparity_interpolation(G, T, O, param, D_it);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << std::setw(50) << std::left << "Elapsed Time for 'disparity_interpolation': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
		
		// show matrix G
		//showG(I_l, G, param, "G");

		// show disparity from disparity_interpolation
		//showDisparity(I_l_c, D_it, "Disparity interpolated");

		
		// execute 'cost_evaluation' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		cost_evaluation(I_l_c, I_r_c, D_it, G, O, param, C_it, census_l, census_r);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << std::setw(50) << std::left << "Elapsed Time for 'cost_evaluation': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;	

		// initialize C_g and C_b new for every iteration
		int sz_g[] = {param.H_bar, param.W_bar, 4}; // dimension of C_g
		int sz_b[] = {param.H_bar, param.W_bar, 3}; // dimension of C_b

		C_g = cv::Mat(3, sz_g, CV_32F, cv::Scalar::all(0));
		cv::Mat C_dummy = cv::Mat(3, sz_g, CV_32F, cv::Scalar::all(0)); // dummy array
		C_b = cv::Mat(3, sz_b, CV_32F, cv::Scalar::all(0));

		for (int vvv = 0; vvv < param.H_bar; vvv++){
			for (int uuu = 0; uuu < param.W_bar; uuu++){
				C_g.at<float>(vvv, uuu, 3) = param.t_lo;
				C_b.at<float>(vvv, uuu, 2) = param.t_hi;
			}
		}


		// execute 'disparity_refinement' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		disparity_refinement(D_it, C_it, G, O, D_f, C_f, C_g, C_b, param);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << std::setw(50) << std::left << "Elapsed Time for 'disparity_refinement': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
		
		// Prepare for next iteration, if not last iteration
		if (i != param.n_iters) {

			// execute 'support_resampling' with elapsed time estimation
			lastTime = boost::posix_time::microsec_clock::local_time();
			support_resampling(C_g, C_b, S, param, I_l_c, I_r_c, census_l, census_r);
			elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
			std::cout << std::setw(50) << std::left << "Elapsed Time for 'support_resampling': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
		
			// execute 'delaunay_triangulation' with elapsed time estimation
			lastTime = boost::posix_time::microsec_clock::local_time();
			delaunay_triangulation(S, G, T, E);
			elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
			std::cout << std::setw(50) << std::left << "Elapsed Time for 'delaunay_triangulation': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;

			// set all support points in G to -1
			for (int j=0; j<S.rows; ++j){
				int u = S.at<float>(j,0);
				int v = S.at<float>(j,1);
				G.at<int>(v,u) = -1;
			}

			// show matrix G
			//showG(I_l, G, param, "G7");

			// refine gridsize
			param.sz_occ = param.sz_occ / 2;

			// update grid height and width
			param.H_bar = int(std::floor(param.H / param.sz_occ));
			param.W_bar = int(std::floor(param.W / param.sz_occ));

			
			// show delaunay grid for i-th iteration
			std::ostringstream oss;
			oss << "Delaunay " << i+1;
			std::string str = oss.str();
			//showGrid(I_l_c, S, E, str);
			
		}
	}

	boost::posix_time::time_duration algorithm_time_elapsed = (boost::posix_time::microsec_clock::local_time() - algorithm_time_start);

	std::cout << "################################################" << std::endl;
	std::cout << std::setw(50) << std::left << "ALGORITHM TOOK: " << std::right << algorithm_time_elapsed.total_microseconds()/1.0e3 << " ms" << std::endl; 
	std::cout << std::setw(50) << std::left << "WITH A SPEED OF: " << std::right << 1.0e6/algorithm_time_elapsed.total_microseconds() << " Hz" << std::endl;
	std::cout << "################################################" << std::endl;

	std::cout << "NUMBER OF ITERATIONS: " << param.n_iters << std::endl;
	std::cout << "NEW SUPPORT POINTS: " << S.rows - num_S_points_init << " / " << S.rows << std::endl;
	cv::Mat disp_points;
	cv::compare(C_f, param.t_hi, disp_points, cv::CMP_LT);
	int num_points = cv::countNonZero(disp_points*1);
	std::cout << "POINTS WITH DISPARITY: " << num_points;
	std:: cout << ", " << float(num_points)/(I_l.rows*I_l.cols)*100 << "% of image" << std::endl;

	showGrid(I_l_c, S, E, "Final Delaunay");
	//showSupportPts(I_l_c, S, "final Support Points");
	showDisparity(I_l_c, D_f, "Final Disparity");
	// calculate accuracy if correct dataset is given
	if(statistics.acc_calc) computeAccuracy(D_f, filename_disp, statistics);
	

	// fill stats struct as output to main
	statistics.alg_time = algorithm_time_elapsed.total_microseconds()/1.0e3;
	statistics.alg_freq = 1.0e6/algorithm_time_elapsed.total_microseconds();
	statistics.it = param.n_iters;

}




void line2(cv::Mat& img, const cv::Point& start, const cv::Point& end, 
                     const cv::Scalar& c1,   const cv::Scalar& c2) {
    cv::LineIterator iter(img, start, end, 8);

    for (int i = 0; i < iter.count; i++, iter++) {
       double alpha = double(i) / iter.count;
       // note: using img.at<T>(iter.pos()) is faster, but 
       // then you have to deal with mat type and channel number yourself
       img(cv::Rect(iter.pos(), cv::Size(1, 1))) = c1 * (1.0 - alpha) + c2 * alpha;
    }
}



void showGrid(cv::Mat I_l, cv::Mat S, cv::Mat E, std::string str){
	// Draw Triangles and display image
	cv::Mat I_triangles = I_l.clone();
	cv::cvtColor(I_triangles, I_triangles, CV_GRAY2RGB);

	float maxDisp, minDisp = 0;
	for (int i=0; i<S.rows; ++i) {
		if (S.at<float>(i,2) > maxDisp) maxDisp = S.at<float>(i,2);
	}

	int k = 0;
	for (int i = 0; i < E.rows/2; ++i) {
		int i1 = E.at<int>(k++,0);
		int i2 = E.at<int>(k++,0);

		cv::Point p1(S.at<float>(i1,0), S.at<float>(i1,1));
		cv::Point p2(S.at<float>(i2,0), S.at<float>(i2,1));

		 // staehlii: THIS IS ACTUALLY NOT A CONSTANT BUT I DON'T WANT TO SEARCH FOR THE CORRECT VALUE
		float scaledDisp1 = S.at<float>(i1,2)/maxDisp;
		float scaledDisp2 = S.at<float>(i2,2)/maxDisp;
		cv::Vec3b color1;
		cv::Vec3b color2;
		if(scaledDisp1 < 0.5)
			color1 = cv::Vec3b(0, scaledDisp1*512, 255);
		else color1 = cv::Vec3b(0, 255, (1-scaledDisp1)*512);

		if(scaledDisp2 < 0.5)
			color2 = cv::Vec3b(0, scaledDisp2*512, 255);
		else color2 = cv::Vec3b(0, 255, (1-scaledDisp2)*512);
		line2(I_triangles, p1, p2, (cv::Scalar) color1, (cv::Scalar) color2);
		//std::cout << "drew line: " << i1 << ", " << i2 << std::endl;
	}
	cv::imshow(str, I_triangles);
}



void showG (cv::Mat I_l, cv::Mat G, parameters param, std::string str){

	std::cout << "showG" << std::endl;
	cv::Mat G_img = I_l.clone();
	

	// loop over all pixels
	for (int u = 0; u < param.W; ++u){
		for (int v = 0; v < param.H; ++v){

			//check if G(u,v) == -1
			if (G.at<int>(v, u, 0) == -1){ // G.at<int>(u, v) == -1
				uchar & color = G_img.at<uchar>(v,u);
				color = 0;
				G_img.at<uchar>(v,u) = color;

			} else{
				uchar & color = G_img.at<uchar>(v,u);
				color = 255;
				G_img.at<uchar>(v,u) = color;

			}
		}
	}

	cv::imshow(str, G_img);
	cv::waitKey(0);
}


void showDisparity(cv::Mat I_l, cv::Mat D_it, std::string str){
		cv::Mat disparity = I_l.clone();
		cv::cvtColor(disparity, disparity, CV_GRAY2RGB);
		cv::Mat normalized;
		normalize(D_it.clone(), normalized, 0, 256, CV_MINMAX, CV_32F);

		for (int x = 0; x < I_l.rows; ++x){
			for (int y = 0; y < I_l.cols; ++y){
				float scaledDisp = normalized.at<float>(x,y)/255.0;
				cv::Vec3b color;
				cv::Point point;
				point.x = y;
				point.y = x;
				if(scaledDisp < 0.5)
					color = cv::Vec3b(0, scaledDisp*512, 255);
				else color = cv::Vec3b(0, 255, (1-scaledDisp)*512);

				if (scaledDisp != 0)
				circle(disparity, point, 1, (cv::Scalar) color, 1);
			}
		}

		cv::imshow(str, disparity);
}

void showSupportPts(cv::Mat I_l, cv::Mat S_it, std::string str){
	cv::Mat support = I_l.clone();
	cv::cvtColor(support, support, CV_GRAY2RGB);

	// loop over all points
	for (int i = 0; i < S_it.rows; ++i){
		cv::Point p1(S_it.at<float>(i,0), S_it.at<float>(i,1));
		float maxDisp = 64.0; // staehlii: THIS IS ACTUALLY NOT A CONSTANT BUT I DON'T WANT TO SEARCH FOR THE CORRECT VALUE
		float scaledDisp1 = S_it.at<float>(i,2)/maxDisp;
		cv::Vec3b color1;

		cv::circle(support, p1, 1, cv::Scalar(0, scaledDisp1*512, 255), 2, 8, 0);

		cv::imshow(str, support);
	}
}



void computeAccuracy(cv::Mat D_f, cv::String filename_disp, stats &statistics){

	if (filename_disp == " ") return;

	std::cout << "################################################" << std::endl;
	std::cout << "ACCURACY (comparing with ground-truth disparity)" << std::endl;
	std::cout << "################################################" << std::endl;

	cv::Mat D_gt_png = cv::imread(filename_disp);

	int D_gt_png_width = D_f.cols;
	int D_gt_png_height = D_f.rows;
	int offset_u = 5;
	int offset_v = 2;
	cv::Rect roi; // region of interest
	roi.x = offset_u;
	roi.y = offset_v;
	roi.width = 1216;
	roi.height = 368;

	cv::Mat D_gt_png_c = D_gt_png(roi);
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
	
	// comparison with ground-truth disparity and 4 different threshold
	for (int32_t v=0; v<D_gt_png_height; v++) {
		for (int32_t u=0; u<D_gt_png_width; u++) {

			gt_value = D_gt_png_c.at<uint16_t>(v,u,0);

			if (D_f.at<float>(v,u) != 0 && gt_value != 0) { // both ground-truth and estimate disparity valid
				n_loops = n_loops + 1;	
				// ground-truth disparity computed by dividing pixel value by 256 
				if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_5)  
					counter_5 = counter_5 + 1;
					if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_4)
						counter_4 = counter_4 + 1;
						if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_3)
							counter_3 = counter_3 + 1;
							if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_2)
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