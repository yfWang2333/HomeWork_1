== SolvePnP 说明文件 ==

文件：
 0.png - 19.png 	素材，已拍摄好的带有棋盘格的图像
 calibrationSession.mat	MATLAB运行结果
 OpenCV_SolvePnP.exe	编译好的可执行文件
 Main.cpp		主程序文件
 Functions.cpp		自编函数文件
 SolvePnP.h		头文件
 01.pdf			内参、外参学习笔记
 02.pdf			畸变学习笔记

EXE 使用说明：
无参数运行：查找同目录下的png文件，编号0-19，不显示校正后的素材文件
带参数运行：	/NotSampled
			必须是第一个参数，启动自带的拍摄功能，照片保存在同目录下
		/DoUndistortion
			必须是第二个参数，显示校正后的素材文件
		如果只开启第二个功能，则第一个参数随便写

运行速度：
Debug模式：大约45s完成参数求解
Release模式：大约5s完成参数求解