#include "image_gradient.hpp"

// image_gradient
//##########################################
//
// input:
// - image_in       : grayscale image [H, W]
// - image_out      : gradient image [H, W]
//
// This function calculates for each pixel the value delta u and delta v (du, dv), using the
// values of the pixels at u+-1 and v+-1. If the pixel lies at the border, the value of the border is used.
// The resulting gradient is calculated by sqrt(du^2 + dv^2)
void image_gradient (cv::Mat &image_in, cv::Mat &image_out, parameters &param){

    // get image size
    int W = image_in.cols;
    int H = image_in.rows;

    //  prepare output
    image_out = cv::Mat(H, W, CV_8UC1);
    // loop over image
    for (int u = 0; u < W; ++u){
        for (int v = 0; v < H; ++v){

            // calculate du using BORDER_REPLICATE
            int du;
            if (u == 0){
                // u at left border
                du = image_in.at<uchar>(v, u + 1) - image_in.at<uchar>(v, u);
            } else 
            if (u == W - 1){
                // u at right border
                du = image_in.at<uchar>(v, u) - image_in.at<uchar>(v, u - 1);
            } else{
                // u within image
                du = image_in.at<uchar>(v, u + 1) - image_in.at<uchar>(v, u - 1);
            }
            


            // calculate dv using BORDER_REPLICATE
            int dv;
            if (v == 0){
                // v at upper border
                dv = image_in.at<uchar>(v + 1, u) - image_in.at<uchar>(v, u);
            } else
            if (v == H - 1){
                // v at lower border
                dv = image_in.at<uchar>(v, u) - image_in.at<uchar>(v - 1, u);
            } else{
                // v within image
                dv = image_in.at<uchar>(v + 1, u) - image_in.at<uchar>(v - 1, u);
            }


            // combine gradients
            int grad = std::sqrt(std::pow(du, 2) + std::pow(dv, 2));

            // write gradint
            image_out.at<uchar>(v, u) = grad;
        }
    }
}