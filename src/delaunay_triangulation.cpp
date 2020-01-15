#include "delaunay_triangulation.hpp"
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Hypertun_SR
{
	
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


/// 上底水平三角形线性扫描填充
/// v1, v2为水平上底, v3为下顶点
bool fillTopFlatTriangulate (cv::Mat &G, const int &idx, 
                             const cv::Point3f &v1, const cv::Point3f &v2, const cv::Point3f &v3)
{
    bool bVerticelV1 = (v1.x==v3.x);
    bool bVerticalV2 = (v2.x==v3.x);
    float k13_inv = 1.;
    float k23_inv = 1.;
    if ( !bVerticelV1 ) k13_inv = (v3.x-v1.x) / (v3.y-v1.y);
    if ( !bVerticalV2 ) k23_inv = (v3.x-v2.x) / (v3.y-v2.y);
    float curr_x_v1 = v3.x;
    float curr_x_v2 = v3.x;
    for (int curr_y=v3.y; curr_y>=v2.y; curr_y--)
    {
        curr_x_v1 = bVerticelV1 ? v1.x : curr_x_v1-k13_inv;
        curr_x_v2 = bVerticalV2 ? v2.x : curr_x_v2-k23_inv;
        int x_min = std::max( std::min(curr_x_v1, curr_x_v2), 0.0f );
        int x_max = std::min( std::max(curr_x_v1, curr_x_v2), (float)(G.cols-1) );
        for (int curr_x=x_min; curr_x<=x_max; curr_x++)
            G.at<int>(curr_y, curr_x) = idx;
    }
    return true;
}


/// 下底水平三角形线性扫描填充
/// v1为上顶点, v2, v3为水平下底
bool fillBottomFlatTriangulate (cv::Mat &G, const int &idx, 
                                const cv::Point3f &v1, const cv::Point3f &v2, const cv::Point3f &v3)
{
    // 算两侧腰的斜率倒数, x方向扫描边界的关于y的单位增量
    bool bVerticelV2 = (v1.x==v2.x);
    bool bVerticalV3 = (v1.x==v3.x);
    float k12_inv = 1.;
    float k13_inv = 1.;
    if ( !bVerticelV2 ) k12_inv = (v2.x-v1.x) / (v2.y-v1.y); 
    if ( !bVerticalV3 ) k13_inv = (v3.x-v1.x) / (v3.y-v1.y);
    float curr_x_v2 = v1.x;
    float curr_x_v3 = v1.x;
    for (int curr_y=v1.y; curr_y<=v2.y; curr_y++)
    {
        // 更新两腰的边界
        curr_x_v2 = bVerticelV2 ? v2.x : curr_x_v2+k12_inv;
        curr_x_v3 = bVerticalV3 ? v3.x : curr_x_v3+k13_inv;
        // 填充
        int x_min = std::max( std::min(curr_x_v2, curr_x_v3), 0.0f );
        int x_max = std::min( std::max(curr_x_v2, curr_x_v3), (float)(G.cols-1) );
        for (int curr_x=x_min; curr_x<=x_max; curr_x++)
            G.at<int>(curr_y, curr_x) = idx;
    }
    return true;
}



/// 线性扫描光栅化算法
/// reference: http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html#algo2
bool rasterizeTriangulate (cv::Mat &G, const int &idx, 
						   const cv::Point3f &v1, const cv::Point3f &v2, const cv::Point3f &v3)
{
    std::vector<cv::Point3f> vec_vtxs{v1, v2, v3};
    std::sort(vec_vtxs.begin(), vec_vtxs.end(), 
              [&](const cv::Point3f &lhs, const cv::Point3f &rhs){ 
                    if (lhs.y==rhs.y) return lhs.x < rhs.x;  // 如果y相等, 按x进行排序
                    return lhs.y < rhs.y; 
              });
    // 本身就是下底水平三角形
    if (vec_vtxs[1].y == vec_vtxs[2].y) 
        return fillBottomFlatTriangulate(G, idx, vec_vtxs[0], vec_vtxs[1], vec_vtxs[2]);
    // 本身就是上底水平三角形
    else if (vec_vtxs[0].y == vec_vtxs[1].y)
        return fillTopFlatTriangulate(G, idx, vec_vtxs[0], vec_vtxs[1], vec_vtxs[2]);
    // 一般三角形
    cv::Point3f hori_vtx;
    hori_vtx.x = vec_vtxs[0].x + (vec_vtxs[1].y-vec_vtxs[0].y) / (vec_vtxs[2].y-vec_vtxs[0].y) * (vec_vtxs[2].x-vec_vtxs[0].x);
    hori_vtx.y = vec_vtxs[1].y;
    hori_vtx.z = 0;  // 还没有做视差平面插值, 随便给

    return fillTopFlatTriangulate(G, idx, vec_vtxs[1], hori_vtx, vec_vtxs[2]) && 
           fillBottomFlatTriangulate(G, idx, vec_vtxs[0], vec_vtxs[1], hori_vtx);
}



cv::Point3f minCoordinate(const cv::Point3f &a, const cv::Point3f &b, const cv::Point3f &c){

	cv::Point3f res;

	if (a.x < b.x) res.x = a.x;
	else res.x = b.x;
	if (c.x < res.x) res.x = c.x;

	if (a.y < b.y) res.y = a.y;
	else res.y = b.y;
	if (c.y < res.y) res.y = c.y;

	return res;
}

cv::Point3f maxCoordinate(const cv::Point3f &a, const cv::Point3f &b, const cv::Point3f &c){

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
#ifdef HPTSR_DEBUG
	std::cout << "delaunay_triangulation.cpp" << std::endl;
#endif 
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

#ifdef HPTSR_DEBUG
	boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::local_time();
#endif 
	char parameters[] = "zQBne";
	triangulate(parameters, &in, &out, NULL);

#ifdef HPTSR_DEBUG
	boost::posix_time::time_duration elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for 'pure triangulation': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
#endif


	// Compute 4D plane parameters
	T = cv::Mat(4, out.numberoftriangles, CV_64F, 0.0);

#ifdef HPTSR_DEBUG    
	lastTime = boost::posix_time::microsec_clock::local_time();
#endif

	// Assign each pixel to the corresponding triangle
	k = 0;
	for (int i = 0; i < out.numberoftriangles; ++i) {
		// tictagtoc.tic();
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
		T.at<float>(3,i) = n_plane.dot(pts[p1]);
		// tictagtoc.toc("compute_plane");


		// Extract 3 vertices from triangle
		cv::Point3f v1 = pts[out.trianglelist[k]];
		cv::Point3f v2 = pts[out.trianglelist[k+1]];
		cv::Point3f v3 = pts[out.trianglelist[k+2]];

		// Extract points of rectangular bounding-box around triangle to reduce search space
		cv::Point3f min = minCoordinate(v1, v2, v3);
		cv::Point3f max = maxCoordinate(v1, v2, v3);

        //! 光栅化: 线性扫描法
        // cv::Mat G_tmp = G.clone();
        rasterizeTriangulate(G, i, v1, v2, v3);

		//! original: 重心坐标系插值算法
		// for(int x = min.x; x < max.x; x++){
		// 	for( int y = min.y; y < max.y; y++){
		// 		cv::Point3f pt;
		// 		pt.x = x;
		// 		pt.y = y;

		// 		if (PointInTriangle(pt, v1, v2, v3)) {
		// 			G.at<int>(y,x,0) = i;
		// 		}
		// 	}
		// }

        // cv::Mat G_cmp;
        // cv::compare(G, G_tmp, G_cmp, CV_CMP_EQ);
        // cv::convertScaleAbs(G_cmp, G_cmp);
        // cv::imshow("G_compare", G_cmp);

		k += 3;
		// tictagtoc.toc("allocate_point");

	}

#ifdef HPTSR_DEBUG
	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << std::setw(50) << std::left << "Elapsed Time for 'compute triangulation plane': " << std::right << elapsed.total_microseconds()/1.0e3 << " ms" << std::endl;
	// tictagtoc.log(std::cout) << std::endl;
    // double G_min, G_max;
    // cv::minMaxIdx(G, &G_min, &G_max);
    // std::cout << "G min value: " << G_min << std::endl;
    // std::cout << "G max value: " << G_max << std::endl;
#endif
	
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
    free(out.neighborlist);
    free(out.edgelist);
}

} // namespace Hypertun_SR
