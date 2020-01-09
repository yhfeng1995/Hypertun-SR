#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

struct parameters {
	// Occupancy grid size used for re-sampling
	int sz_occ;

	// Number of iterations the algorithm is allowed to run
	int n_iters;

	// Lower and upper threshold for validating disparities
	float t_lo;
	float t_hi;

    // image height and width
    int H;
    int W;

	// grid height and width
	int H_bar;
	int W_bar;

	// number of pixels with high gradient
	int nOfHiGradPix;

	// threshold for image gradient
	int im_grad;

	// threshold for epipolar search
	float t_epi;
	// window size for epipolar serch
	int epi_window;
};

#endif // PARAMETERS_HPP