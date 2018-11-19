#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <opencv/cxeigen.hpp>

#define PI 3.14159

using namespace std;
using namespace cv;
using namespace Eigen;

vector<cv::Point2f> Generate2DPoints(); //生成２Ｄ点容器，即像素坐标系下的点集
vector<cv::Point3f> Generate3DPoints(); //生成３Ｄ空间点容器，世界坐标系下的点集

int main( int argc, char* argv[])
{
    // Read points
    vector<cv::Point2f> imagePoints = Generate2DPoints();
    /**
     * [x1, y1;
     *  x2, y2;
     *  .
     *  .
     *  .
     *  xn, yn]
     */
    vector<cv::Point3f> objectPoints = Generate3DPoints();
    /**
     * [x1, y1, z1;
     *  x2, y2, z2;
     *  .
     *  .
     *  .
     *  xn, yn, z3]
     */

    cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << endl;

    //camera parameters　相机内参
    double fx = 810.1; //focal length x
    double fy = 805.62; //focal le

    double cx = 915.85; //optical centre x
    double cy = 520.38; //optical centre y

    Mat cameraMatrix(3,3,cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0)=fx;
    cameraMatrix.at<double>(1,1)=fy;
    cameraMatrix.at<double>(2,2)=1;
    cameraMatrix.at<double>(0,2)=cx;
    cameraMatrix.at<double>(1,2)=cy;
    cameraMatrix.at<double>(0,1)=0;
    cameraMatrix.at<double>(1,0)=0;
    cameraMatrix.at<double>(2,0)=0;
    cameraMatrix.at<double>(2,1)=0;
    /**
     * 生成内参矩阵，{fx, 0, cx,
     *               0, fy, cy,
     *               0, 0, 1 }
     *
     */

    cout << "Initial cameraMatrix: " << cameraMatrix << endl;

    Mat distCoeffs(4,1,cv::DataType<double>::type); //定义畸变系数[r1, r2, k1, k2], r为径向畸变,　k为切向畸变
    distCoeffs.at<double>(0) = -0.36151;
    distCoeffs.at<double>(1) = 0.19181;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    Mat rvec(3,1,DataType<double>::type); //定义储存旋转向量和平移向量
    Mat tvec(3,1,DataType<double>::type);

    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl;

    Mat Rvec;
    Mat_<float> Tvec;

    rvec.convertTo(Rvec, CV_32F);    //旋转向量格式转换
    tvec.convertTo(Tvec, CV_32F);   //平移向量格式转换

    Mat_<float> rotMat(3, 3);
    Rodrigues(Rvec, rotMat);  //将旋转向量变换成旋转矩阵

    cout << "rotMat: " << rotMat << endl;
    cout << "Tvec: " << Tvec << endl;

    //根据旋转矩阵求出坐标旋转角
    double theta_x = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
    double theta_y = atan2(-rotMat.at<double>(2, 0),
                    sqrt(rotMat.at<double>(2, 1)*rotMat.at<double>(2, 1) + rotMat.at<double>(2, 2)*rotMat.at<double>(2, 2)));
    double theta_z = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
    /**
    * https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
     * 旋转矩阵转化成坐标旋转角
    */

    //将弧度转化为角度
    theta_x = theta_x * (180 / PI);
    theta_y = theta_y * (180 / PI);
    theta_z = theta_z * (180 / PI);

    cout << "theta_x: " << theta_x << endl;
    cout << "theta_y: " << theta_y << endl;
    cout << "theta_z: " << theta_z << endl;

    /**
     * P_camera = R*P_world + T
     * P_camera = 0 时，　P_world = - inverse(R) * T
     * 求得相机在世界坐标系中的坐标
     */

    Matrix<float, Dynamic, Dynamic> R_n;
    Matrix<float, Dynamic, Dynamic> T_n;
    cv2eigen(rotMat, R_n);
    cv2eigen(Tvec, T_n);
    Vector3f P_oc;

    P_oc = -R_n.inverse()*T_n;
    cout << "世界坐标" << P_oc << endl;

    vector<Point2f> projectedPoints;
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << endl;
    }

    return 0;
}


vector<Point2f> Generate2DPoints()
{
    vector<Point2f> points;

    float x,y;

    x=970.5;y=151.5;
    points.push_back(Point2f(x,y));

    x=232;y=640;
    points.push_back(Point2f(x,y));

    x=1232;y=990;
    points.push_back(Point2f(x,y));

    x=1693;y=243;
    points.push_back(Point2f(x,y));

    for(unsigned int i = 0; i < points.size(); ++i)
    {
        cout << points[i] << endl;
    }

    return points;
}


vector<Point3f> Generate3DPoints()
{
    vector<Point3f> points;
    float x, y, z;
    x=0.0; y=0.0; z=0.0;
    points.push_back(Point3f(x,y,z));

    x=297.0; y=0.0; z=0.0;
    points.push_back(Point3f(x,y,z));

    x=297.0; y=210.0; z=0.0;
    points.push_back(Point3f(x,y,z));

    x=0.0;y=2210.0;z=0.0;
    points.push_back(Point3f(x,y,z));

    for(unsigned int i = 0; i < points.size(); ++i)
    {
        cout << points[i] << endl;
    }

    return points;
}

