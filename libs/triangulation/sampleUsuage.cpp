computeDelaunayTriangulation() {
	// input/output structure for triangulation
	struct triangulateio in, out;
	int32_t k;

	// inputs
	in.numberofpoints = static_cast<int>(m_vSupportPts.size());
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));
	k = 0;

	for (int32_t i = 0; i<m_vSupportPts.size(); i++) {
		in.pointlist[k++] = m_vSupportPts[i].pt(0, 0);
		in.pointlist[k++] = m_vSupportPts[i].pt(1, 0);
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
	
	// do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
	char parameters[] = "zQBn";
	triangulate(parameters, &in, &out, NULL);

	// put resulting triangles into vector tri
	m_vTriangles.clear();
	k = 0;
	for (int32_t i = 0; i< out.numberoftriangles; i++) {
		//get the index of the triangle vertices first, and remove that triangle if it has a long edge
		// get the three supporting vertex of current triangle
		Support_pt v1 = m_vSupportPts.at(out.trianglelist[k]);
		Support_pt v2 = m_vSupportPts.at(out.trianglelist[k + 1]);
		Support_pt v3 = m_vSupportPts.at(out.trianglelist[k + 2]);

		// if the triangle is good, register it.
		m_vTriangles.push_back(Triangle(out.trianglelist[k], out.trianglelist[k + 1], out.trianglelist[k + 2]));
		// register neigbouring triangles
		m_vTriangles.at(i).n1 = out.neighborlist[k];
		m_vTriangles.at(i).n2 = out.neighborlist[k + 1];
		m_vTriangles.at(i).n3 = out.neighborlist[k + 2];
		// register the index of current triangle
		m_vTriangles.at(i).index = i;
		
		//register the center of the triangle in pixel coordinate
		Eigen::Vector2d center = v1.pt + v2.pt + v3.pt;
		center /= 3.0f;
		m_vTriangles.at(i).center = center;
		// compute the surface normal of the 3 triangle face made of center point of unit sphere + 2 points from the triangle on unit sphere
		Eigen::Vector3d ray1 = v1.ray;
		Eigen::Vector3d ray2 = v2.ray;
		Eigen::Vector3d ray3 = v3.ray;

		m_vTriangles.at(i).fn1 = ray2.cross(ray3);
		m_vTriangles.at(i).fn2 = ray3.cross(ray1);
		m_vTriangles.at(i).fn3 = ray1.cross(ray2);
		
		// register the pixels which is contained by the outer-circle of current triangle, as an initial guess for the point belonging to which triangle
		registerTriangle2Pixels(m_vTriangles.at(i));

		// compute plane parameters
		computeTrianglePlaneParameters(m_vTriangles.at(i));

		// update index for for loop
		k += 3;
	}
	// free memory used for triangulation
	free(in.pointlist);
	free(out.pointlist);
	free(out.trianglelist);
}