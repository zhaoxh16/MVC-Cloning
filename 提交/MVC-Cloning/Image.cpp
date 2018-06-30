#include "Image.h"

Image::Image() {

}

void Image::readImage(string source, string target, Point offset) {
	this->sourceImage = imread(source);
	this->targetImage = imread(target);
	this->offset = offset;
}

void Image::init() {
	getContour();
	triangulate();
	computeMVC();
	clone();
	showMesh();
}

void Image::show() {
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
	findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
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
		Eigen::Vector3d pTarget(vertices[i].x+eps, vertices[i].y+eps, 0);
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
			double cos2a = v0.dot(v1) / (sqrt(v0.dot(v0))*sqrt(v1.dot(v1)));
			double cos2b = v1.dot(v2) / (sqrt(v1.dot(v1))*sqrt(v2.dot(v2)));
			if (cos2a > 1.0)cos2a = 1.0;
			if (cos2a < -1.0)cos2a = -1.0;
			if (cos2b > 1.0) cos2b = 1.0;
			if (cos2b < -1.0)cos2b = -1.0;
			double a = fabs(acos(cos2a)/2.0);
			double b = fabs(acos(cos2b)/2.0);
			double tana = tan(a);
			double tanb = tan(b);
			double w = (tana + tanb) / (sqrt(v1.dot(v1)) + eps);
			mvcVector.push_back(w);
			sum += w;
		}
		for (int j = 0; j < mvcVector.size(); ++j) {
			mvcVector[j] /= sum;
		}
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

void Image::clone() {
	vector<Eigen::Vector3d> diff;
	for (int i = 0; i < contour.size(); ++i) {
		Vec3b tar = targetImage.at<Vec3b>(contour[i].y + offset.y, contour[i].x + offset.x);
		Vec3b source = sourceImage.at<Vec3b>(contour[i].y, contour[i].x);
		Eigen::Vector3d colorDiff((int)tar[0] - (int)source[0], (int)tar[1] - (int)source[1], (int)tar[2] - (int)source[2]);
		diff.push_back(colorDiff);
	}
	cDiff = new Eigen::Vector3d*[sourceImage.rows];
	for (int i = 0; i < sourceImage.rows; ++i) {
		cDiff[i] = new Eigen::Vector3d[sourceImage.cols];
		for (int j = 0; j < sourceImage.cols; ++j)
			cDiff[i][j] = Eigen::Vector3d(0, 0, 0);
	}

	for (int i = 0; i < vertices.size(); ++i) {
		Eigen::Vector3d colorDiff(0, 0, 0);
		for (int k = 0; k < diff.size(); ++k) {
			colorDiff += diff[k] * mvcMatrix[i][k];
		}
		cDiff[vertices[i].y][vertices[i].x] = colorDiff;
	}

	for (int i = 0; i < meshs.size(); ++i) {
		for (int j = 0; j < meshs[i].insidePoints.size(); ++j) {
			Eigen::Vector3d colorDiff(0, 0, 0);
			for (int k = 0; k < 3; ++k) {
				colorDiff += meshs[i].weight[j][k] * cDiff[meshs[i].vertices[k].y][meshs[i].vertices[k].x];
			}
			cDiff[meshs[i].insidePoints[j].y][meshs[i].insidePoints[j].x] = colorDiff;
		}
	}

	for (int i = 0; i < sourceImage.rows; ++i) {
		for (int j = 0; j < sourceImage.cols; ++j) {
			Vec3b sImage = sourceImage.at<Vec3b>(i, j);
			int pixel0 = max(0.0, min(255.0, cDiff[i][j][0] + (int)sImage[0]));
			int pixel1 = max(0.0, min(255.0, cDiff[i][j][1] + (int)sImage[1]));
			int pixel2 = max(0.0, min(255.0, cDiff[i][j][2] + (int)sImage[2]));
			if (computed[i][j]) targetImage.at<Vec3b>(i + offset.y, j + offset.x) = Vec3b(pixel0,pixel1,pixel2);
		}
	}
	imshow("target", targetImage);
	waitKey(0);
}
