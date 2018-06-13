#include "Image.h"

Image::Image() {

}

void Image::readImage(string source, string target) {
	this->sourceImage = imread(source);
	this->targetImage = imread(target);
}

void Image::init() {
	getContour();
	triangulate();
	computeMVC();
	/*showMesh();*/
}

void Image::show() {
	for (int i = 0; i < contour.size(); ++i) {
		cout << contour[i] << endl;
	}
	imshow("contour", mask);
	waitKey(0);
}

void Image::save(string file) {

}

void Image::getContour() {
	cvtColor(sourceImage, mask, COLOR_BGR2GRAY);
	vector<vector<Point>>contours;
	vector<Vec4i> hierarchy;
	Mat tmpImage;
	GaussianBlur(mask, tmpImage, Size(3, 3), 0);
	Canny(tmpImage, tmpImage, 100, 250);
	findContours(tmpImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
	int index = 0;
	int length = 0;
	for (int i = 0; i < contours.size(); ++i) {
		if (contours[i].size() > length) {
			index = i;
			length = contours[i].size();
		}
	}
	vector<Point> tmpContour = contours[index];
	for (int i = 0; i < tmpContour.size(); ++i) {
		if (find(contour.begin(), contour.end(), tmpContour[i]) == contour.end())
			contour.push_back(tmpContour[i]);
	}
}

void Image::triangulate() {
	triangulation(contour,vertices, meshs, mask);
	for (int i = 0; i < meshs.size(); ++i) {
		for (int k = 0; k < 3; ++k) {
			int index = find(vertices.begin(), vertices.end(), meshs[i].vertices[k]) - vertices.begin();
			meshs[i].verticesNumber.push_back(index);
		}
	}
}

void Image::computeMVC() {
	for (int i = 0; i < vertices.size(); ++i) {
		vector<double> mvcVector;
		Eigen::Vector3d pTarget(vertices[i].x, vertices[i].y, 0);
		double sum = 0;
		for (int j = 0; j < contour.size(); ++j) {
			Eigen::Vector3d pPre, pNow, pNext;
			pNow = Eigen::Vector3d(contour[j].x, contour[j].y, eps);
			if (j == 0)pPre = Eigen::Vector3d(contour[contour.size() - 1].x, contour[contour.size() - 1].y, eps);
			else pPre = Eigen::Vector3d(contour[j - 1].x, contour[j - 1].y, eps);
			if (j == contour.size() - 1) pNext = Eigen::Vector3d(contour[0].x, contour[0].y, eps);
			else pNext = Eigen::Vector3d(contour[j + 1].x, contour[j + 1].y, eps);
			Eigen::Vector3d v0 = pPre - pTarget;
			Eigen::Vector3d v1 = pNow - pTarget;
			Eigen::Vector3d v2 = pNext - pTarget;
			double a = acos(v0.dot(v1) / (sqrt(v0.dot(v0))*sqrt(v1.dot(v1))))/2.0;
			double b = acos(v1.dot(v2) / (sqrt(v1.dot(v1))*sqrt(v2.dot(v2))))/2.0;
			double tana = fabs(sin(a) / cos(a));
			double tanb = fabs(sin(b) / cos(b));
			double w = (tana + tanb) / (sqrt((pNow - pTarget).dot(pNow - pTarget)) + eps);
			mvcVector.push_back(w);
			sum += w;
		}
		for (int j = 0; j < mvcVector.size(); ++j)
			mvcVector[j] /= sum;
		mvcMatrix.push_back(mvcVector);
	}

	computed = new bool*[sourceImage.rows];
	for (int i = 0; i < sourceImage.rows; ++i)
		computed[i] = new bool[sourceImage.cols];
	for (int i = 0; i < sourceImage.rows; ++i) {
		for (int j = 0; j < sourceImage.cols; ++j) {
			computed[i][j] = false;
		}
	}
	for (int i = 0; i < meshs.size(); ++i) {
		for (int j = 0; j < 3; ++j) {
			computed[meshs[i].vertices[j].y][meshs[i].vertices[j].x] = true;
		}
		int xMin = min(min(meshs[i].vertices[0].x, meshs[i].vertices[1].x), meshs[i].vertices[2].x);
		int xMax = max(max(meshs[i].vertices[0].x, meshs[i].vertices[1].x), meshs[i].vertices[2].x);
		int yMin = min(min(meshs[i].vertices[0].y, meshs[i].vertices[1].y), meshs[i].vertices[2].y);
		int yMax = max(max(meshs[i].vertices[0].y, meshs[i].vertices[1].y), meshs[i].vertices[2].y);
		for (int x = xMin; x <= xMax; ++x) {
			for (int y = yMin; y <= yMax; ++y){
				Eigen::Vector3d coords;
				Point p(x, y);
				if (computed[y][x] == false && insideTriangle(p,meshs[i].vertices[0],meshs[i].vertices[1],meshs[i].vertices[2],coords)) {
					computed[y][x] = true;
					meshs[i].insidePoints.push_back(p);
					meshs[i].weight.push_back(coords);
				}
			}
		}
	}
}

bool Image::insideTriangle(Point& p, Point& p0, Point& p1, Point &p2, Eigen::Vector3d& coords) {
	Point v0(p.x - p0.x, p.y - p0.y);
	Point v1(p.x - p1.x, p.y - p1.y);
	Point v2(p.x - p2.x, p.y - p2.y);

	Point v10(p1.x - p0.x, p1.y - p0.y);
	Point v21(p2.x - p1.x, p2.y - p1.y);
	Point v02(p0.x - p2.x, p0.y - p2.y);

	int cross0 = v0.x * v10.y - v10.x * v0.y;
	int cross1 = v1.x * v21.y - v21.x * v1.y;
	int cross2 = v2.x * v02.y - v02.x * v2.y;
	int crossA = v10.x * v02.y - v02.x * v10.y;

	coords.z() = (double)cross0 / crossA;
	coords.x() = (double)cross1 / crossA;
	coords.y() = (double)cross2 / crossA;

	return cross0 <= 0 && cross1 <= 0 && cross2 <= 0 ||
		cross0 >= 0 && cross1 >= 0 && cross2 >= 0;
}

void Image::showMesh() {
	Mat meshImage = Mat::zeros(mask.size(), CV_8UC1);
	for (int i = 0; i < meshs.size(); ++i) {
		for (int j = 0; j < 3; ++j) {
			meshImage.at<uchar>(meshs[i].vertices[j].y, meshs[i].vertices[j].x) = 255;
			Point pNow = meshs[i].vertices[j];
			Point pNext;
			if (j == 2) pNext = meshs[i].vertices[0];
			else pNext = meshs[i].vertices[j + 1];
			//以下是画直线
			int x0 = pNow.x;
			int x1 = pNext.x;
			int y0 = pNow.y;
			int y1 = pNext.y;
			int x, y, dx, dy, e;
			bool flag = 0;
			dx = x1 - x0, dy = y1 - y0;
			if (abs(dy) > abs(dx))flag = 1, swap(dx, dy), swap(x0, y0);
			e = -abs(dx);
			x = x0, y = y0;
			for (int i = 0; i <= abs(dx); i++)
			{
				if (!flag)
					meshImage.at<uchar>(y,x) = 255;
				else meshImage.at<uchar>(x,y) = 255;
				x += dx > 0 ? 1 : -1, e += 2 * abs(dy);
				if (e >= 0)y += dy > 0 ? 1 : -1, e -= 2 * abs(dx);
			}
		}
	}
	imshow("meshImage", meshImage);
	waitKey(0);
}
