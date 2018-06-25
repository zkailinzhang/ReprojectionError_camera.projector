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

	cv::Point2f point2, point1;
	std::vector<cv::Point2f> camera_corners;
	std::vector<cv::Point2f> projector_corners;
	std::vector<std::vector<cv::Point2f> > camera_corners_active;
	std::vector<std::vector<cv::Point2f> > projector_corners_active;
	FILE * fp1;
	FILE * fp2;

	char fileName[50] = "0";

	for (int i = 1; i < 5; i++)
	{
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

	cv::Mat R;
	cv::Mat T;
	cv::FileStorage fsrt("calibration.yml", cv::FileStorage::READ);
	if (!fsrt.isOpened()) {
		return 0;
	}
	fsrt["R"] >> R;
	fsrt["T"] >> T;
	fsrt.release();
	Mat RT(4, 4, R.type(),Scalar(0));
	
	for (int i = 0; i < R.cols; i++)
	{
		for (int j = 0; j < R.rows; j++)
			RT.at< double >(i, j) = R.at< double >(i, j);
	}			
	RT.at< double >(0, 3) = T.at< double >(0, 0);
	RT.at< double >(1, 3) = T.at< double >(1, 0);
	RT.at< double >(2, 3) = T.at< double >(2, 0);
	RT.at< double >(3, 3) = 1;
	RT.at< double >(3, 0) = 0;
	RT.at< double >(3, 1) = 0;
	RT.at< double >(3, 2) = 0;


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


	Mat imagePoints(3,3,CV_32FC2);
	Mat objectPoints(3, 3, CV_32FC2);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			imagePoints.at<Vec2f>(i, j)[0] = 340 * (i + 1); 
			imagePoints.at<Vec2f>(i, j)[1] = 340 * (j + 1);
			//imagePoints.at<Vec3f>(i, j)[2] = 1.0;
		}
	}

	//Point2f ok //Point3f  error
	cv::perspectiveTransform(imagePoints, objectPoints, homo);
	//cv::transform(objectPoints, objectPoints2, T);
	//cvTransform(imagePoints, objectPoints, R,T);
	//objectPoints += T;


	//cv::transform(imagePoints, objectPoints, RT);

	//打开投影仪在图片上画16个圆圈，投出去
	cv::Mat pro_img(cv::Size(1024, 768), CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			//cout << objectPoints.at<Vec3f>(i, j)[0] << " " << objectPoints.at<Vec3f>(i, j)[2];
			cv::circle(pro_img, cv::Point2f(objectPoints.at<Vec2f>(i, j)[0],
				objectPoints.at<Vec2f>(i, j)[1] ), 10, CV_RGB(255, 0, 0), 3, 8, 0);
		}
	}

	//在qt gui中不能用啊
	cv::namedWindow("pro", CV_WINDOW_AUTOSIZE);
	cv::imshow("pro", pro_img);
	cv::waitKey(50);
	cv::moveWindow("pro", 1024, 0);
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
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cv::circle(cam_img, cv::Point2f(imagePoints.at<Vec2f>(i, j)[0],
				imagePoints.at<Vec2f>(i, j)[1] ), 10, CV_RGB(0, 0, 255), 3, 8, 0);
		}
	}
	cv::imshow("cam", cam_img);
	cv::waitKey();
	cv::imwrite("cam_img_16.png", cam_img);



	return 0;
}

