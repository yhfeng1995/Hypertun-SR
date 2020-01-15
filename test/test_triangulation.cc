#include <random>
#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
#include "../ThirdParty/triangulation/DelaunayTriangulation.h"

int main(int argc, char **argv)
{
    if (argc!=2 && argc!=4)
    {
        cerr << "Usage: ./bin/test_triangulation num_vtxs [width height]" << endl;
        return 1;
    }
    int num_vtxs = atoi(argv[1]);
    int width=640, height=480;
    if (argc==4)
    {
        width = atoi(argv[2]);
        height = atoi(argv[3]);
    }

    random_device rd;
    uniform_real_distribution<float> distrib_x(0, width-1);
    uniform_real_distribution<float> distrib_y(0, height-1);

    triangulateio tri_in, tri_out;
    // in
    tri_in.pointlist = new float[3*num_vtxs];
    tri_in.pointattributelist = NULL;
    tri_in.pointmarkerlist = NULL;
    tri_in.numberofpoints = num_vtxs;
    tri_in.numberofpointattributes = 0;
    tri_in.segmentlist = NULL;
    tri_in.segmentmarkerlist = NULL;
    tri_in.numberofsegments = 0;
    tri_in.holelist = NULL;
    tri_in.numberofholes = 0;
    tri_in.regionlist = NULL;
    tri_in.numberofregions = 0;

    // out
    tri_out.pointlist = NULL;
    tri_out.pointattributelist = NULL;
    tri_out.pointmarkerlist = NULL;
    tri_out.numberofpoints = 0;
    tri_out.trianglelist = NULL;
    tri_out.triangleattributelist = NULL;
    tri_out.trianglearealist = NULL;
    tri_out.neighborlist = NULL;
    tri_out.numberoftriangles = 0;
    tri_out.numberofcorners = 0;
    tri_out.numberoftriangleattributes = 0;
    tri_out.segmentlist = NULL;
    tri_out.segmentmarkerlist = NULL;
    tri_out.numberofsegments = 0;
    tri_out.edgelist = NULL;
    tri_out.edgemarkerlist = NULL;
    tri_out.numberofedges = 0;

    for (int i=0; i<num_vtxs*2; i+=2)
    {
        tri_in.pointlist[i] = distrib_x(rd);
        tri_in.pointlist[i+1] = distrib_y(rd);
    }

    char parameters[] = "zQne";
    triangulate(parameters, &tri_in, &tri_out, NULL);
    
    cv::Mat im(height, width, CV_8UC3, cv::Scalar(127, 127, 127));
    for (int i=0; i<num_vtxs*2; i+=2)
    {
        cv::Point pt(tri_out.pointlist[i], tri_out.pointlist[i+1]);
        cv::circle(im, pt, 2, cv::Scalar(0, 255, 0));
        cv::rectangle(im, cv::Rect(cv::Point(pt.x-2, pt.y-2), cv::Point(pt.x+2, pt.y+2)), cv::Scalar(0, 255, 0));
    }
    for (int i=0; i<tri_out.numberofedges*2; i+=2)
    {
        int idx1 = tri_out.edgelist[i];
        int idx2 = tri_out.edgelist[i+1];
        cv::Point pt1(tri_out.pointlist[idx1*2], tri_out.pointlist[idx1*2+1]);
        cv::Point pt2(tri_out.pointlist[idx2*2], tri_out.pointlist[idx2*2+1]);
        cv::line(im, pt1, pt2, cv::Scalar(255, 0, 0));
    }
    for (int i=0; i<tri_out.numberoftriangles*3; i+=3)
        cout << "no." << i/3 
             << ": n1=" << tri_out.neighborlist[i] 
             << "; n2=" << tri_out.neighborlist[i+1]
             << "; n3=" << tri_out.neighborlist[i+2] << endl;
    cv::imshow("Delaunay", im);
    cv::waitKey();

    delete tri_in.pointlist;
    free(tri_out.trianglelist) ;
    free(tri_out.neighborlist) ;
    free(tri_out.edgelist) ;
}