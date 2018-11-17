#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define ERR_VCAPTURE_NOCAMERA 5001
#define ERR_VCAPTURE_NOCBOARD 5002
#define ERR_VCAPTURE_NOIMAGEC 5003
#define ERR_VCAPTURE_NOIMAGEL 5004
#define FUNC_MAIN_EXIT_HALF 6001
#define SET_DOUBLE_FUNC_TCRITERIA_ACCURACY 1
#define SET_ARR_COLOUR_A CV_RGB(255,69,0)
#define	SET_ARR_COLOUR_B CV_RGB(0,255,127)
#define SET_ARR_COLOUR_C CV_RGB(0,191,255)
#define SET_ARR_COLOUR_D CV_RGB(255,0,255)
#define SET_INT_CBOARD_COLS 12
#define SET_INT_CBOARD_GSIZE 200	/*TO BE MODIFIED*/
#define SET_INT_CBOARD_PNTIDX 38	/*TO BE MODIFIED*/
#define SET_INT_CBOARD_ROWS 9
#define SET_INT_FUNC_IMSHOW_DELAY 1500
#define SET_INT_FUNC_TCRITERIA_NITER 20	/*TO BE MODIFIED*/
#define SET_INT_PICTURE_MAXSAMPLE 20
#define SET_INT_VCAPTURE_WIDTH 1280
#define SET_INT_VCAPTURE_HEIGHT 720
#define SET_INT_WINDOW_HSIZE 10		/*TO BE MODIFIED*/
#define SET_STRING_WINDOW_TITLE "Solve PnP"
#define RETURN(ERR_CODE) system("pause > nul");return ERR_CODE;

extern int Index;
extern Mat FrameCurrent;

vector<Point3f> CalcPointCamera(int GridCols, int GridRows, double SizeGrid);
vector<vector<Point3f>> CalcPointCamera(int CountImage, int GridCols, int GridRows, double SizeGrid);