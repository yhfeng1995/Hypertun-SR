#include "delaunay_triangulation.hpp"

float sign (cv::Point3f p1, cv::Point3f p2, cv::Point3f p3) {
    return (p1.y - p3.y) * (p2.x - p3.x) - (p2.y - p3.y) * (p1.x - p3.x);
}

bool PointInTriangle (cv::Point3f pt, cv::Point3f v1, cv::Point3f v2, cv::Point3f v3) {
    bool b1, b2, b3;

    b1 = sign(pt, v1, v2) <= 0.0f;
    b2 = sign(pt, v2, v3) <= 0.0f;
    b3 = sign(pt, v3, v1) <= 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

cv::Point3f minCoordinate(cv::Point3f a, cv::Point3f b, cv::Point3f c){

	cv::Point3f res;

	if (a.x < b.x) res.x = a.x;
	else res.x = b.x;
	if (c.x < res.x) res.x = c.x;

	if (a.y < b.y) res.y = a.y;
	else res.y = b.y;
	if (c.y < res.y) res.y = c.y;

	return res;
}

cv::Point3f maxCoordinate(cv::Point3f a, cv::Point3f b, cv::Point3f c){

	cv::Point3f res;

	if (a.x > b.x) res.x = a.x;
	else res.x = b.x;
	if (c.x > res.x) res.x = c.x;

	if (a.y > b.y) res.y = a.y;
	else res.y = b.y;
	if (c.y > res.y) res.y = c.y;

	return res;
}

// delaunay_triangulation:
// inputs: 
// - S: Nx3 matrix with N points and [u,v,d] for each one
// - H: Height of image
// - W: Width of image
//
// outputs:
// - G: HxW matrix with and index for each pixel to the corresponding triangle
// - T: 4xnum_triangles matrix with the plane parameters for each triangle
// - E: 1x(2*num_edges) matrix with 2 points for each edge used to plot


void delaunay_triangulation(cv::Mat &S, cv::Mat &G, cv::Mat &T, cv::Mat &E){

	std::cout << "delaunay_triangulation.cpp" << std::endl;

	// Store support points into input variable
	int N = S.rows;

	struct triangulateio in, out;
	in.numberofpoints = static_cast<int>(N);
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));

	cv::Point3f pts[N];
	int dummy_array[N];
	int k = 0;
	for (int i = 0; i < N; ++i) {
		in.pointlist[k++] = S.at<float>(i,0);
		in.pointlist[k++] = S.at<float>(i,1);
		pts[i].x = S.at<float>(i,0);
		pts[i].y = S.at<float>(i,1);
		pts[i].z = S.at<float>(i,2);
	}

	in.numberofpointattributes = 0;
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofsegments = 0;
	in.numberofholes = 0;
	in.numberofregions = 0;
	in.regionlist = NULL;

	// outputs
	out.pointlist = NULL;
	out.pointattributelist = NULL;
	out.pointmarkerlist = NULL;
	out.trianglelist = NULL;
	out.triangleattributelist = NULL;
	out.neighborlist = NULL;
	out.segmentlist = NULL;
	out.segmentmarkerlist = NULL;
	out.edgelist = NULL;
	out.edgemarkerlist = NULL;

	char parameters[] = "zQBne";
	triangulate(parameters, &in, &out, NULL);



	// Compute 4D plane parameters
	T = cv::Mat(4, out.numberoftriangles, CV_64F, 0.0);

	// Assign each pixel to the corresponding triangle
	k = 0;
	for (int i = 0; i < out.numberoftriangles; ++i) {
		cv::Point3f line_12, line_13;

		// Take indices of the 3 triangle vertices
		int p1 = out.trianglelist[k];
		int p2 = out.trianglelist[k+1];
		int p3 = out.trianglelist[k+2];
		//k += 3;

		// Construct 2 lines consisting of triangle edges
		line_12 = pts[p1] - pts[p2];
		line_13 = pts[p1] - pts[p3];

		// Get vector orthogonal to plane using cross product
		cv::Point3f n_plane = line_12.cross(line_13);
		
		// Compute euclidean norm and normalize vector
		float norm = sqrt(n_plane.x * n_plane.x + 
						  n_plane.y * n_plane.y +
						  n_plane.z * n_plane.z);
		n_plane.x /= norm;
		n_plane.y /= norm;
		n_plane.z /= norm;

		// Compute missing plane parameter
		T.at<float>(0,i) = n_plane.x;
		T.at<float>(1,i) = n_plane.y;
		T.at<float>(2,i) = n_plane.z;
		T.at<float>(3,i) = n_plane.x * pts[p1].x +
						   n_plane.y * pts[p1].y + 
						   n_plane.z * pts[p1].z;


		// Extract 3 vertices from triangle
		cv::Point3f v1 = pts[out.trianglelist[k]];
		cv::Point3f v2 = pts[out.trianglelist[k+1]];
		cv::Point3f v3 = pts[out.trianglelist[k+2]];

		// Extract points of rectangular bounding-box around triangle to reduce search space
		cv::Point3f min = minCoordinate(v1, v2, v3);
		cv::Point3f max = maxCoordinate(v1, v2, v3);


		for(int x = min.x; x < max.x; x++){
			for( int y = min.y; y < max.y; y++){
				cv::Point3f pt;
				pt.x = x;
				pt.y = y;

				if (PointInTriangle(pt, v1, v2, v3)) {
					G.at<int>(y,x,0) = i;
				}
			}
		}

		k += 3;

	}

	

	k = 0;
	E = cv::Mat(2 * out.numberofedges, 1, CV_32S);
	for (int i = 0; i < out.numberofedges; ++i)
	{
		// For plotting triangle edges
		E.at<int>(k, 0) = out.edgelist[k];
		E.at<int>(k + 1, 0) = out.edgelist[k + 1];
		k += 2;
		int bla = 0;

	}

	free(in.pointlist);
	free(out.pointlist);
	free(out.trianglelist);

}