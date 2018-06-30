#include "triangulation.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "mesh.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

using namespace std;
using namespace cv;

// External classes and structs
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;

typedef CDT::Vertex_handle Vertex_handle;
typedef K::Point_2 CGALPoint;
typedef CDT::Finite_vertices_iterator VIter;
typedef CDT::Finite_faces_iterator FIter;

void triangulation(vector<Point>& boundaryPoints,vector<Point>& vertices, vector<Mesh>& meshs, const Mat& mask) {
	vector<CGALPoint> cBoundaryPoints;
	for (int i = 0; i < boundaryPoints.size(); ++i) {
		cBoundaryPoints.push_back(CGALPoint(boundaryPoints[i].x, boundaryPoints[i].y));
	}

	CDT constraints;
	for (int i = 0; i < cBoundaryPoints.size(); ++i) {
		if (i == cBoundaryPoints.size() - 1) {
			Vertex_handle handle1 = constraints.insert(cBoundaryPoints[i]);
			Vertex_handle handle2 = constraints.insert(cBoundaryPoints[0]);
			constraints.insert_constraint(handle1, handle2);
		}
		else {
			Vertex_handle handle1 = constraints.insert(cBoundaryPoints[i]);
			Vertex_handle handle2 = constraints.insert(cBoundaryPoints[i + 1]);
			constraints.insert_constraint(handle1, handle2);
		}
	}

	Mesher myMesher(constraints);
	myMesher.refine_mesh();

	vertices.clear();
	VIter vIter;
	for (vIter = constraints.finite_vertices_begin(); vIter != constraints.finite_vertices_end(); ++vIter) {
		vertices.push_back(Point(vIter->point().hx(), vIter->point().hy()));
	}
	FIter fIter;
	for (fIter = constraints.finite_faces_begin(); fIter != constraints.finite_faces_end(); ++fIter) {
		Mesh mesh(Point(fIter->vertex(0)->point().hx(), fIter->vertex(0)->point().hy()), Point(fIter->vertex(1)->point().hx(), fIter->vertex(1)->point().hy()), Point(fIter->vertex(2)->point().hx(), fIter->vertex(2)->point().hy()));
		double middleX = (mesh.vertices[0].x + mesh.vertices[1].x + mesh.vertices[2].x) / 3;
		double middleY = (mesh.vertices[0].y + mesh.vertices[1].y + mesh.vertices[2].y) / 3;
		if (mask.at<uchar>(middleY, middleX) == 0 || mask.at<uchar>(mesh.vertices[0].y, mesh.vertices[0].x) == 0 || mask.at<uchar>(mesh.vertices[1].y, mesh.vertices[1].x) == 0 || mask.at<uchar>(mesh.vertices[2].y, mesh.vertices[2].x) == 0)
			continue;
		meshs.push_back(mesh);
	}
}
