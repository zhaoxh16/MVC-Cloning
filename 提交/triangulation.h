#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "mesh.h"
using namespace std;

void triangulation(vector<Point>& boundaryPoints, vector<Point>& vertices, vector<Mesh>& meshs, const Mat& mask);