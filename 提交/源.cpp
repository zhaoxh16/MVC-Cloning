#include "Image.h"

int main() {
	Image image;
	image.readImage("ship.bmp","target2.jpg",Point(250,200));
	image.init();
	return 0;
}