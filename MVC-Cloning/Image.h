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
	void readImage(string source, string target);
	void init();
	void show();
	void save(string file);

private:
	Mat sourceImage;
	Mat targetImage;
	Mat mask;
	vector<Point> contour;//��¼�߽���λ��
	vector<Point> vertices;//��¼������Ƭ�����λ��
	vector<Mesh> meshs;//��¼������Ƭ
	vector<vector<double>> mvcMatrix;
	bool** computed;//ĳ�����Ƿ��Ѿ����������

	void getContour();
	void triangulate();
	void computeMVC();
	void showMesh();
	bool insideTriangle(Point& p, Point& p0, Point& p1, Point& p2, Eigen::Vector3d& coords);
}; 