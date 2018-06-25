// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include<stdio.h>
#include<stdlib.h>

using namespace std;
using namespace cv;

int main()
{

//H
	cv::Point2f point2, point1;
	std::vector<cv::Point2f> camera_corners;
	std::vector<cv::Point2f> projector_corners;
	std::vector<std::vector<cv::Point2f> > camera_corners_active;
	std::vector<std::vector<cv::Point2f> > projector_corners_active;
	FILE * fp1;
	FILE * fp2;
	char fileName[50] = "0";

	for (int i = 1; i < 5; i++){
		sprintf(fileName, "%02d", i);
		string fn1, fn2;
		string str(fileName);
		fn1 += "cam_" + str + ".txt";
		fn2 += "proj_" + str + ".txt";
		fp1 = fopen(fn1.c_str(), "r");
		fp2 = fopen(fn2.c_str(), "r");
		if (!fp2 || !fp1) {return 0;}
		for (unsigned j = 0; j < 88; j++)
		{
			fscanf(fp1, "%f %f\n", &point2.x, &point2.y);
			fscanf(fp2, "%f %f\n", &point1.x, &point1.y);
			// 			cout << point2.x << " " << point2.y; 
			// 			printf("%f %f\n", point1.x,point1.y); ok
			camera_corners.push_back(point2);
			projector_corners.push_back(point1);
		}
		camera_corners_active.push_back(camera_corners);
		projector_corners_active.push_back(projector_corners);
		projector_corners.clear();
		camera_corners.clear();
	}
	fclose(fp1);
	fclose(fp2);
		//zkl begin
	//没用findhomograph 平面对平面，论文一直避免全局单应，但这个透视变换没问题，对应对之间的变换，不考虑了标定板平面不平面？
	cv::Mat homo(3, 3, CV_32FC1);
	homo = cv::findHomography(camera_corners_active.at(3), projector_corners_active.at(3), cv::RANSAC);
	cv::Mat transform(3, 3, CV_32FC1);
	//transform = cv::getPerspectiveTransform(camera_corners_active.at(0), projector_corners_active.at(0));
	cout << homo << endl << transform;
	cv::FileStorage fs("perspec_homoMat.xml", cv::FileStorage::WRITE);
	if (!fs.isOpened()) {		return 0;}
	fs << "homo" << homo << "perspecMat" << transform;
	fs.release();

	//Point2f ok //Point3f  error
	
	//cv::transform(objectPoints, objectPoints2, T);
	//cvTransform(imagePoints, objectPoints, R,T);
	//objectPoints += T;



	//cv::perspectiveTransform(imagePoints, objectPoints, homo);
	//cv::transform(imagePoints, objectPoints, RT);

//P
	cv::Mat camK,camKc,projK,projKc;
	cv::FileStorage fsK("calibrationyou.yml", cv::FileStorage::READ);
	if (!fsK.isOpened()) {
		return 0;
	}
	fsK["cam_K"] >> camK;
	fsK["cam_kc"] >> camKc;
	fsK["proj_K"] >> projK;
	fsK["proj_kc"] >> projKc;
	fsK.release();

	cv::Mat cam_rvecs = cv::Mat::zeros(1, 3, CV_32F);
	cv::Mat proj_rvecs = cv::Mat::zeros(1, 3, CV_32F);
	cv::Mat camR,camT,projR,projT;
	cv::FileStorage fsrt("cam_pro_ExRT_lastyou.xml", cv::FileStorage::READ);
	if (!fsrt.isOpened()) {
		return 0;
	}
	fsrt["cam_rvecs"] >> cam_rvecs;
	fsrt["cam_tvecs"] >> camT;
	fsrt["proj_rvecs"] >> proj_rvecs;
	fsrt["proj_tvecs"] >> projT;
	fsrt.release();
	
	cv::Rodrigues(cam_rvecs, camR);
	cv::Rodrigues(proj_rvecs, projR);



	//pixel  uv
	Mat imagePoints(16,2,CV_32FC1); 
	Mat projPoints(16,2,CV_32FC1); 
	//mm
	Mat objectPoints(16, 3, CV_32FC1);  
	for (int i = 0; i < 16; i++)
	{
		objectPoints.at<float>(i, 0) = 300+60 * (i%4); 
		objectPoints.at<float>(i, 1) = 240+60 * (i/4);
		objectPoints.at<float>(i, 2) = 0.0;
	}
	Mat imagePoints2(16,3,CV_32FC1);
	//cvMatMulAdd();
// 	gemm(camR,objectPoints,1,camT,1,imagePoints2);
// 	perspectiveTransform();
// 	cvMatMulAdd();
// 	cv::transform(imagePoints2, imagePoints2, camK);

	projectPoints(objectPoints,cam_rvecs,camT,camK,camKc,imagePoints);

	//perspectiveTransform(imagePoints, objectPoints, homo);  只验证相机内参，投影用单应
	cv::projectPoints(objectPoints, proj_rvecs,projT,projK,projKc,projPoints);

	//打开投影仪在图片上画16个圆圈，投出去
	cv::Mat pro_img(cv::Size(1024, 768), CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < 16; i++) {
			//cout << objectPoints.at<Vec3f>(i, j)[0] << " " << objectPoints.at<Vec3f>(i, j)[2];
			cv::circle(pro_img, cv::Point2f(projPoints.at<float>(i,0),
				projPoints.at<float>(i, 1) ), 10, CV_RGB(255, 0, 0), 3, 8, 0);
	}

	//有边框//在qt gui中不能用啊
	//cv::namedWindow("pro", CV_WINDOW_AUTOSIZE);	
	//cv::setWindowProperty("pro", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	
	//有边框 组合使用
	cv::namedWindow("pro", CV_WINDOW_NORMAL);
	cv::setWindowProperty("pro", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	//cv::setWindowProperty("pro", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	cv::Mat img = imread("pat_00.png", 1);
	cv::imshow("pro", img);
	cv::waitKey(50);
	cv::moveWindow("pro", 1280, 0);
// 	HWND win_handle = FindWindow(0, L"input");
// 	if (!win_handle)
// 	{
// 		printf("Failed FindWindow\n");
// 	}
// 
// 	// Resize  
// 	unsigned int flags = (SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOMOVE | SWP_NOZORDER);
// 	flags &= ~SWP_NOSIZE;
// 	unsigned int x = 0;
// 	unsigned int y = 0;
// 	unsigned int w = input.cols;
// 	unsigned int h = input.rows;
// 	SetWindowPos(win_handle, HWND_NOTOPMOST, x, y, w, h, flags);
// 
// 	// Borderless  
// 	SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
// 	ShowWindow(win_handle, SW_SHOW);



	cv::imwrite("pro_img_16.png", pro_img);

	//相机采集，在这张图上画定义的16个圆圈 看是否重合
	cv::waitKey(30);
	cv::VideoCapture cap(0);            
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	// while(true)
	cv::waitKey(60);
	cv::Mat cam_img(cv::Size(1920, 1080), CV_8UC3);
	cap >> cam_img;

	cv::namedWindow("cam", 0);
	cv::imshow("cam", cam_img);
	cv::waitKey(30);
	for (int i = 0; i < 16; i++) {
			cv::circle(cam_img,cv::Point2f(imagePoints.at<float>(i,0),
				imagePoints.at<float>(i, 1) ), 10, CV_RGB(0, 0, 255), 3, 8, 0);
	}
	cv::imshow("cam", cam_img);
	cv::waitKey();
	cv::imwrite("cam_img_16.png", cam_img);



	return 0;
}

