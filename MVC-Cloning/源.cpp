#include "Image.h"

int main() {
	Image image;
	image.readImage("D:\\OneDrive\\Tsinghua\\2018Spring\\计算机图形学\\ship.bmp","D:\\OneDrive\\Tsinghua\\2018Spring\\计算机图形学\\target2.jpg",Point(900,210));
	image.init();
	return 0;
}