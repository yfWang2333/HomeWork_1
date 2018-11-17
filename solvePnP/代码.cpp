#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
Vec3f rotationMatrixToEulerAngles(Mat &R);
int main()
{
	double a[10];
	a[0] = 1.1478e+03;
	a[1] = 1.1504e+03;
	a[2] = 7.1955e+02;
	a[3] = 5.3973e+02;
	a[4] = 0.1616;
	a[5] = -0.7004;
	a[6] = 0;
	a[7] = 0;

	Mat camera_matrix = (Mat_<double>(3, 3) << a[0], 0, a[2], 0, a[1], a[3], 0, 0, 1);
	Mat distortion_coeffs = (Mat_<double>(1, 4) << a[4], a[5], a[6], a[7]);

	vector<Point3f> Points3D;
	Points3D.push_back(cv::Point3f(0, 0, 0));
	Points3D.push_back(cv::Point3f(168, 0, 0));
	Points3D.push_back(cv::Point3f(168, 105, 0));
	Points3D.push_back(cv::Point3f(0, 105, 0));

	vector<cv::Point2f> Points2D;
	/*Points2D.push_back(cv::Point2f(1067.7,787.9));
	Points2D.push_back(cv::Point2f(594.3,854.8));
	Points2D.push_back(cv::Point2f(566.5,529.7));
	Points2D.push_back(cv::Point2f(1034.3,492.7));*/

	/*Points2D.push_back(cv::Point2f(804,720));
	Points2D.push_back(cv::Point2f(393,732));
	Points2D.push_back(cv::Point2f(396,480));
	Points2D.push_back(cv::Point2f(792,469));*/

	Points2D.push_back(cv::Point2f(900, 681));
	Points2D.push_back(cv::Point2f(655, 664));
	Points2D.push_back(cv::Point2f(672,594));
	Points2D.push_back(cv::Point2f(889,609));
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	solvePnP(Points3D, Points2D, camera_matrix, distortion_coeffs, rvec, tvec, false, CV_ITERATIVE);

	double rm[9];
	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);

	Vec3f eluer = rotationMatrixToEulerAngles(rotM);
	cout << eluer*57;//z,y,x
	cout << "\n\n";
	cout << tvec;//y,x,z
	getchar();
	return 0;
}

bool isRotationMatrix(Mat &R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;

}

Vec3f rotationMatrixToEulerAngles(Mat &R)
{

	assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3f(x, y, z);
}