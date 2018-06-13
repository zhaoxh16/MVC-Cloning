#include "Image.h"

int main() {
	Image image;
	image.readImage("D:\\OneDrive\\Tsinghua\\2018Spring\\计算机图形学\\bear.jpg","");
	image.init();
	image.show();
	return 0;
}