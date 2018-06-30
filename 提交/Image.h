#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "mesh.h"
#include "triangulation.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace cv;

const double eps = 0.000001;

class Image {
public:
	Image();
	void readImage(string source, string target, Point offset);
	void init();
	void show();
	void save(string file);

private:
	Mat sourceImage;
	Mat targetImage;
	Mat mask;
	Point offset;
	vector<Point> contour;//记录边界点的位置
	vector<Point> vertices;//记录三角面片顶点的位置
	vector<Mesh> meshs;//记录三角面片
	vector<vector<double>> mvcMatrix;
	Eigen::Vector3d** cDiff;
	bool** computed;//某个点是否已经被计算过了

	void getContour();
	void triangulate();
	void computeMVC();
	void showMesh();
	bool insideTriangle(Point& p, Point& p0, Point& p1, Point& p2, Eigen::Vector3d& coords);
	void clone();
}; 