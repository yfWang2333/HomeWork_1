// CameraCalibration_example.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <opencv2/core/core.hpp>//核心功能模块
#include <opencv2/highgui/highgui.hpp> //高层GUI图形用户界面
#include <opencv2/imgproc/imgproc.hpp>//OpenCV图像处理头文件
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <iostream>
#define pi 3.14159265358979323846


using namespace std;
using namespace cv;

vector<Point3f> CalBoardCornerPositions(int boardW, int boardH) 
{
	vector<Point3f> ObjectPoints;
	for (int i = 0; i < boardH; i++)
	{
		for (int j = 0; j < boardW; j++)
		{
			ObjectPoints.push_back(Point3f(float(j * 36), float(i * 36), 0));
		}
	}
	return ObjectPoints;
}

// PipeLine: 相机标定->外部参照点测量->外部参照点获取->solvepnp姿态解算->验证计算值世界测量值差距
int main()
{
	ifstream fin("calibdata.txt"); // 标定所用图像文件的路径 
	// Mat ImageInput = imread("1.jpg");
	int image_count = 0;
	Size image_size; // 图像尺寸
	Size board_size = Size(9, 6); // 棋盘格上每行、列的角点数
	vector<Point2f> image_points_buf; // 缓存每幅图像上检测到的角点
	vector<vector<Point2f>> image_points_seq; // 保存检测到的角点
	string filename;

	while (getline(fin, filename))
	{ 
		image_count++;
		cout << "image_count = " << image_count << endl;
		Mat ImageInput = imread(filename);

		// 0.读入第一张图片的宽高信息
		if (image_count == 1)
		{
			image_size.height = ImageInput.rows;
			image_size.width = ImageInput.cols;
			cout << "width = " << image_size.width << " " << " height = " << image_size.height << endl;
		}

		// 1.提取角点
		if (0 == findChessboardCorners(ImageInput, board_size, image_points_buf))
		{
			cout << "cannot find chessboard corners" << endl;
			exit(1);
		}
		else
		{
			Mat grayImage;
			cvtColor(ImageInput, grayImage, CV_RGB2GRAY);
			// 亚像素精确化
			find4QuadCornerSubpix(grayImage, image_points_buf, Size(14, 14)); // 对粗提取的角点精确化
			image_points_seq.push_back(image_points_buf); // 保存亚像素角点
			// 在图像上显示角点位置
			drawChessboardCorners(grayImage, board_size, image_points_buf, true); // 用于在图像中标记角点
			// imshow("Camera Calibration", grayImage);
			// waitKey(0);
		}
	}

	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int cornerNum = board_size.width * board_size.height; // 图片上的角点数(54)
	
	//显示第一张的角点坐标
	/*
	for (int j = 0; j < cornerNum; j++)
	{
		cout << "(" << image_points_seq[0][j].x << "," << image_points_seq[0][j].y << ")" << endl;
	}
	*/

	// 2.像机标定
	Size square_size = Size(36, 36); // 实际测量的标定板上每个棋盘格的大小（mm）
	vector<vector<Point3f>> object_points; // 保存标定板上角点的三维坐标
	
	//内外参数
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 像机内参数矩阵
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 像机的5个畸变系数：k1, k2, p1, p2, k3
	vector<Mat> tvecsMat; // 平移向量
	vector<Mat> rvecsMat; // 旋转向量
	vector<int> point_counts; // 每幅图像中角点的数量

	// 初始化标定板上角点的三维坐标
	for (int k = 0; k < image_count; k++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				Point3f realPoint; // 假设标定板放在世界坐标系中z = 0的平面上
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}

	//初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
	for (int i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}
	// 开始标定
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	// 保存标定结果
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 保存图像的旋转矩阵
	Rodrigues(rvecsMat[0], rotation_matrix);// 将旋转向量转换为相对应的旋转矩阵 
	
	// 输出内外参数
	cout << "内参矩阵：" << cameraMatrix << endl;
	cout << "畸变系数：" << distCoeffs << endl;

	/*
	for (int i = 0; i < image_count; i++)
	{
		cout << "第" << i + 1 << "幅：" << endl;
		cout << "旋转向量：" << rvecsMat[0] << endl;
		cout << "旋转矩阵：" << rotation_matrix << endl;
		cout << "平移向量：" << tvecsMat[0] << endl;
	}
	*/

	VideoCapture cap(0);
	Mat images, gray;
	Mat rvecs(3, 1, CV_32F), tvecs(3, 1, CV_32F), R(3, 3, CV_32FC1, Scalar::all(0));
	float Ang_X, Ang_Y, Ang_Z;
	float X, Y, Z;

	while (1)
	{
		cap >> images;
		vector<Point2f> corners;
		bool found = findChessboardCorners(images, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found)
		{
			cvtColor(images, gray, CV_BGR2GRAY);
			cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)); // 0.1为精度
			drawChessboardCorners(images, board_size, corners, found);
			vector<Point3f> objectPoints = CalBoardCornerPositions(board_size.height, board_size.width);
			solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvecs, tvecs);
			Rodrigues(rvecs, R);

			Ang_X = asin(R.at<double>(1, 0) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
			Ang_Y = asin(-R.at<double>(2, 0)) / pi * 180;
			Ang_Z = asin(R.at<double>(2, 1) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;

			X = R.at<double>(0, 0) * objectPoints[0].x + R.at<double>(0, 1)  * objectPoints[0].y + R.at<double>(0, 2)  * objectPoints[0].z + tvecs.at<double>(0, 0);
			Y = R.at<double>(1, 0) * objectPoints[0].x + R.at<double>(1, 1)  * objectPoints[0].y + R.at<double>(1, 2)  * objectPoints[0].z + tvecs.at<double>(1, 0);
			Z = R.at<double>(2, 0) * objectPoints[0].x + R.at<double>(2, 1)  * objectPoints[0].y + R.at<double>(2, 2)  * objectPoints[0].z + tvecs.at<double>(2, 0);
		 
			cout << "X = " << X << "; Y = " << Y << "; Z = " << Z << endl;
			cout << "Ang_X = " << Ang_X << " ; Ang_Y = " << Ang_Y << " ; Ang_Z = " << Ang_Z << endl;
			imshow("chessboard", images);
		}
		//imshow("chessboard", images);
		waitKey(0);
	}

	return 0;
}
