#include "delaunay_triangulation.hpp"
#include <random>

/// 线性扫描光栅化算法
/// reference: http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html#algo2
bool rasterizeTriangulate (cv::Mat &G, const int &idx, 
						   const cv::Point3f &v1, const cv::Point3f &v2, const cv::Point3f &v3);

int main(int argc, char **argv)
{
    int H = 480;
    int W = 640;
    cv::Mat im = cv::Mat::zeros(H, W, CV_32S);
    // random triangulate
    std::random_device rd;
    std::uniform_real_distribution<float> dist_x(0, 640);
    std::uniform_real_distribution<float> dist_y(0, 480);
    std::vector<cv::Point3f> vec_vtxs(3);
    // for (int i=0; i<3; i++)
    // {
    //     vec_vtxs[i].x = dist_x(rd);
    //     vec_vtxs[i].y = dist_y(rd);
    //     vec_vtxs[i].z = -1;
    // }
    vec_vtxs[0].x = 0; vec_vtxs[0].y = 240;
    vec_vtxs[1].x = 320; vec_vtxs[1].y = 120;
    vec_vtxs[2].x = 639; vec_vtxs[2].y = 479;
    rasterizeTriangulate(im, 255, vec_vtxs[0], vec_vtxs[1], vec_vtxs[2]);
    cv::convertScaleAbs(im, im);
    cv::imshow("test_rastertize", im);
    cv::waitKey(0);
}