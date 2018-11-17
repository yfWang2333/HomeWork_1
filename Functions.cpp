#include "SolvePnP.h"

vector<Point3f> CalcPointCamera(int GridCols, int GridRows, double SizeGrid) {
	vector<Point3f> PointCamera;
	for (int i = 0; i < GridRows; ++i)
		for (int j = 0; j < GridCols; ++j)
			PointCamera.push_back(Point3f(float(j*SizeGrid), float(i*SizeGrid), 0));
	return PointCamera;
}
vector<vector<Point3f>> CalcPointCamera(int CountImage, int GridCols, int GridRows, double SizeGrid) {
	vector<vector<Point3f>> PointCamera(CountImage);
	for (int i = 0; i < CountImage; ++i) {
		PointCamera[i] = vector<Point3f>(0);
		for (int j = 0; j < GridRows; ++j)
			for (int k = 0; k < GridCols; k++)
				PointCamera[i].push_back(Point3f(float(j*SizeGrid), float(k*SizeGrid), 0));
	}
	return PointCamera;
}