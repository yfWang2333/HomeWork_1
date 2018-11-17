#include "SolvePnP.h"

int Index;
Mat FrameCurrent;

int main(int argc, char *argv[]) {
	bool FoundChessboard = false;
	char KeyPressed = 0;
	double ErrorCalibration = 0.0;
	double EulerYawX = 0.0, EulerPitchY = 0.0, EulerRollZ = 0.0;
	double FrameRate = 0.0;
	double PoseWorld[3] = { 0.0 };
	int FrameRefreshInterval = 0;
	int SizeChessboardGrid = SET_INT_CBOARD_GSIZE;
	vector<int> ImageUseful;
	vector<int> SettingPNGSave = { IMWRITE_PNG_COMPRESSION,9 };
	vector<Mat> ImageSampled;
	vector<Mat> VectorRotation, VectorTranslation;
	vector<Point2f> PointGridCorner;
	vector<Point3f> PointGridCamera;
	vector<vector<Point2f>> PointChessboardImage;
	vector<vector<Point3f>> PointChessboardCamera;
	Mat ImageTemp, ImageTempGray;
	Mat ParameterIntrinsic, ParameterDistortion, MatrixRotation;
	Mat VectorRotationS(3, 1, CV_32F), VectorTranslationS(3, 1, CV_32F);
	Size SizeChessboard(SET_INT_CBOARD_COLS - 1, SET_INT_CBOARD_ROWS - 1);
	Size SizeImage;

	cout << "Camera Calibration and Pose Solution\n" << endl;
	cout << "[Info] Connecting camera..." << endl;
	VideoCapture Frame(0);
	if (!Frame.isOpened()) {
		cerr << "[Error] No camera is avalible." << endl;
		RETURN(ERR_VCAPTURE_NOCAMERA);
	}
	cout << "[Info] Adjusting camera";
	Frame.set(CV_CAP_PROP_FRAME_WIDTH, SET_INT_VCAPTURE_WIDTH);
	Frame.set(CV_CAP_PROP_FRAME_HEIGHT, SET_INT_VCAPTURE_HEIGHT);
	FrameRate = Frame.get(CV_CAP_PROP_FPS);
	if (fabs(FrameRate - 0.0) < 1E-6) {
		cout << "..." << endl;
		cerr << "[Error] No image is captured." << endl;
		RETURN(ERR_VCAPTURE_NOIMAGEC);
	}
	printf(" :: %4d x %4d @ %2d FPS\n",
		SET_INT_VCAPTURE_WIDTH, SET_INT_VCAPTURE_HEIGHT, static_cast<int>(FrameRate));
	FrameRefreshInterval = static_cast<int>(1000.0 / FrameRate);
	if (argc > 1 && !strcmp(argv[1], "/NotSampled")) {
		cout << "[Info] Starting capturing..." << endl;
		namedWindow(SET_STRING_WINDOW_TITLE, WINDOW_AUTOSIZE);
		Frame >> FrameCurrent;
		cout << "[Info] Press SPACE to take a new sample and Q/q to quit." << endl;
		printf("[Info] %2d sample(s) remaining.\n", SET_INT_PICTURE_MAXSAMPLE);
		for (; Index < SET_INT_PICTURE_MAXSAMPLE;) {
			Frame >> FrameCurrent;
			KeyPressed = waitKey(FrameRefreshInterval);
			switch (KeyPressed) {
			case ' ':
				imwrite(to_string(Index++) + ".png", FrameCurrent, SettingPNGSave);
				printf("    >> %2d sample(s) remaining.\n", SET_INT_PICTURE_MAXSAMPLE - Index);
				break;
			case 'Q':
			case 'q':
				return FUNC_MAIN_EXIT_HALF;
			default:
				break;
			}
			imshow(SET_STRING_WINDOW_TITLE, FrameCurrent);
		}
		cout << "[Info] Enough samples have been acquired and saved." << endl;
	}
	cout << "[Info] Loading saved samples..." << endl;
	for (int i = 0; i < SET_INT_PICTURE_MAXSAMPLE; ++i)
		ImageSampled.push_back(imread(to_string(i) + ".png"));
	if (ImageSampled.empty() || (!ImageSampled.empty() && !ImageSampled[0].rows)) {
		cerr << "[Error] No image loaded." << endl;
		RETURN(ERR_VCAPTURE_NOIMAGEL);
	}
	cout << "[Info] Processing in progress..." << endl;
	cout << "    >> Finding calibration pattern - chessboard";
	for (int i = 0; i < SET_INT_PICTURE_MAXSAMPLE; ++i) {
		FoundChessboard = findChessboardCorners(ImageSampled[i], SizeChessboard, PointGridCorner,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (FoundChessboard) {
			ImageUseful.push_back(i);
			cvtColor(ImageSampled[i], ImageTempGray, CV_BGR2GRAY);
			cornerSubPix(ImageTempGray, PointGridCorner, Size(SET_INT_WINDOW_HSIZE, SET_INT_WINDOW_HSIZE), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
					SET_INT_FUNC_TCRITERIA_NITER, SET_DOUBLE_FUNC_TCRITERIA_ACCURACY));
			PointChessboardImage.push_back(PointGridCorner);
			drawChessboardCorners(ImageSampled[i], SizeChessboard, PointGridCorner, FoundChessboard);
		}
	}
	if (ImageUseful.empty()) {
		cout << "..." << endl;
		cerr << "[Error] No chessboard is found." << endl;
		RETURN(ERR_VCAPTURE_NOCBOARD);
	}
	else {
		printf(" :: %2llu accepted and %2llu rejected.\n",
			ImageUseful.size(), SET_INT_PICTURE_MAXSAMPLE - ImageUseful.size());
	}
	cout << "    >> Calculating points in camera coordinate..." << endl;
	PointChessboardCamera = CalcPointCamera(PointChessboardImage.size(), SizeChessboard.width,
		SizeChessboard.height, SET_INT_CBOARD_GSIZE);
	SizeImage = ImageSampled[0].size();
	cout << "    >> Calculating intrinsic parameters of camera..." << endl;
	cout << "    >> Calculating extrinsic parameters of saved samples..." << endl;
	ErrorCalibration = calibrateCamera(PointChessboardCamera, PointChessboardImage, SizeImage, ParameterIntrinsic,
		ParameterDistortion, VectorRotation, VectorTranslation);
	if (argc > 2 && !strcmp(argv[2], "/DoUndistortion")) {
		for (int i = 0; i < ImageUseful.size(); ++i) {
			if (i == 0) cout << "[Info] ";
			else cout << "    >> ";
			printf("Displaying undistorted images No.%02d\n", i + 1);
			undistort(ImageSampled[ImageUseful[i]], ImageTemp, ParameterIntrinsic, ParameterDistortion);
			imshow(SET_STRING_WINDOW_TITLE, ImageTemp);
			waitKey(SET_INT_FUNC_IMSHOW_DELAY);
		}
	}
	cout << "[Info] Camera calibration is done." << endl;
	printf("    >> RMS Value: %.2f\n", ErrorCalibration);
	printf("    >> Principal point: %.2f%*c%.2f\n",
		ParameterIntrinsic.at<double>(0, 2), 5, ' ', ParameterIntrinsic.at<double>(1, 2));
	printf("    >> Focal length:    %.2f%*c%.2f\n",
		ParameterIntrinsic.at<double>(0, 0), 5, ' ', ParameterIntrinsic.at<double>(1, 1));
	cout << "    >> Distortion:" << endl;
	for (int i = 0; i < ParameterDistortion.rows; ++i)
		for (int j = 0; j < ParameterDistortion.cols; ++j)
			printf("%*c%.10f\n", 8, ' ', ParameterDistortion.at<double>(i, j));
	cout << "[Info] Pose solution is currently running..." << endl;
	cout << "    >> Fixed circle is the principal point." << endl;
	cout << "    >> The circle approximately in the middle of calibration pattern is the original point." << endl;
	cout << "[Info] Press SPACE to quit." << endl;
	namedWindow(SET_STRING_WINDOW_TITLE, WINDOW_AUTOSIZE);
	while (waitKey(FrameRefreshInterval) != ' ') {
		Frame >> FrameCurrent;
		ImageTemp = FrameCurrent.clone();
		circle(FrameCurrent, Point2f(ParameterIntrinsic.at<double>(0, 2),
			ParameterIntrinsic.at<double>(1, 2)), 10, SET_ARR_COLOUR_D, 2);
		imshow(SET_STRING_WINDOW_TITLE, FrameCurrent);
		FoundChessboard = findChessboardCorners(ImageTemp, SizeChessboard, PointGridCorner,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (FoundChessboard) {
			cvtColor(ImageTemp, ImageTempGray, CV_BGR2GRAY);
			cornerSubPix(ImageTempGray, PointGridCorner, Size(SET_INT_WINDOW_HSIZE, SET_INT_WINDOW_HSIZE), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, SET_INT_FUNC_TCRITERIA_NITER,
					SET_DOUBLE_FUNC_TCRITERIA_ACCURACY));
			drawChessboardCorners(FrameCurrent, SizeChessboard, PointGridCorner, FoundChessboard);
			PointGridCamera = CalcPointCamera(SizeChessboard.width, SizeChessboard.height, SET_INT_CBOARD_GSIZE);
			solvePnP(PointGridCamera, PointGridCorner, ParameterIntrinsic, ParameterDistortion,
				VectorRotationS, VectorTranslationS);
			Rodrigues(VectorRotationS, MatrixRotation);
			EulerYawX = asin(MatrixRotation.at<double>(1, 0) / cos(asin(-MatrixRotation.at<double>(2, 0)))) / M_PI * 180;
			EulerPitchY = asin(-MatrixRotation.at<double>(2, 0)) / M_PI * 180;
			EulerRollZ = asin(MatrixRotation.at<double>(2, 1) / cos(asin(-MatrixRotation.at<double>(2, 0)))) / M_PI * 180;
			for (int i = 0; i < 3; ++i)
				PoseWorld[i] = MatrixRotation.at<double>(i, 0)*PointGridCamera[SET_INT_CBOARD_PNTIDX].x
				+ MatrixRotation.at<double>(i, 1)*PointGridCamera[SET_INT_CBOARD_PNTIDX].y
				+ MatrixRotation.at<double>(i, 2)*PointGridCamera[SET_INT_CBOARD_PNTIDX].z
				+ VectorTranslationS.at<double>(i, 0);
			circle(FrameCurrent, PointGridCorner[SET_INT_CBOARD_PNTIDX], 10, SET_ARR_COLOUR_D, 2);
			putText(FrameCurrent, "X: " + to_string(PoseWorld[0] / 10) + " mm", { 20,50 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_A, 2);
			putText(FrameCurrent, "Y: " + to_string(PoseWorld[1] / 10) + " mm", { 20,150 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_B, 2);
			putText(FrameCurrent, "Z: " + to_string(PoseWorld[2] / 10) + " mm", { 20,250 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_C, 2);
			putText(FrameCurrent, "Yaw:" + to_string(EulerYawX) + " Degree", { 20, 450 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_A, 2);
			putText(FrameCurrent, "Pitch:" + to_string(EulerPitchY) + " Degree", { 20, 550 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_B, 2);
			putText(FrameCurrent, "Roll:" + to_string(EulerRollZ) + " Degree", { 20, 650 }, FONT_HERSHEY_SIMPLEX, 1.0, SET_ARR_COLOUR_C, 2);
			imshow(SET_STRING_WINDOW_TITLE, FrameCurrent);
		}
	}
	destroyWindow(SET_STRING_WINDOW_TITLE);
	cout << "[Info] Press any key to quit." << endl;
	system("pause > nul");
	return 0;
}