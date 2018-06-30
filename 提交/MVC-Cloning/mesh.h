#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace cv;

class Mesh {
public:
	Mesh(Point a, Point b, Point c) {
		vertices.push_back(a);
		vertices.push_back(b);
		vertices.push_back(c);
	}
	vector<Point> vertices;
	vector<int> verticesNumber;
	vector<Point> insidePoints;
	vector<Eigen::Vector3d> weight;

};