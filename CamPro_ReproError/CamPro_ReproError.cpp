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
	
	for (int i = 22; i < 26; i++)
	{
		sprintf(fileName, "%02d", i);

		string fn1, fn2;
		string str(fileName);
		fn1 += "cam_" + str + ".txt";
		fn2 += "proj_" + str + ".txt";
		fp1 = fopen(fn1.c_str(), "r");
		fp2 = fopen(fn2.c_str(), "r");
		if (!fp2 || !fp1) {
			return 0;
		}

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

	//zkl begin
	//没用findhomograph 平面对平面，论文一直避免全局单应，但这个透视变换没问题，对应对之间的变换，不考虑了标定板平面不平面？
	
	
	cv::Mat homo(3, 3, CV_32FC1);
	homo	= cv::findHomography(camera_corners_active.at(1), projector_corners_active.at(1), cv::RANSAC);
	cv::Mat transform(3,3,CV_32FC1);
    //transform = cv::getPerspectiveTransform(camera_corners_active.at(0), projector_corners_active.at(0));
	cout << homo<<endl<<transform;


	cv::FileStorage fs("perspec_homoMat.xml", cv::FileStorage::WRITE);
	if (!fs.isOpened())	{
		return 0;
	}
	fs << "homo" << homo<< "perspecMat" << transform;
	fs.release();


	cv::Point3f point;
	std::vector<cv::Point3f > img_row_Points;
	std::vector<cv::Point3f>  objectPoints;
	std::vector<cv::Point3f> imagePoints;
	for (int i2 = 0; i2 < 4; i2++)
	{
		for (int j = 0; j < 4; j++) {
			point.x = 100 * (i2 + 1); point.y = 100 * (j + 1); point.z = 1.0;
			img_row_Points.push_back(point);
		}

		//        for(int j=0;j<4;j++){
		//            //std::vector<cv::Point3f > img_row_Points;
		//            img_row_Points.push_back(cv::Point3f(100*(i2+1),100*(j+1),1.0));
		//        }
		//迷糊了
// 		imagePoints.push_back(img_row_Points);
// 		img_row_Points.clear();
	}

	//Point2f ok
	//cv::perspectiveTransform(img_row_Points, objectPoints, homo);

	//Point3f  error
	//cv::perspectiveTransform(img_row_Points, objectPoints, R);


		for (int n=0;n<16;n++)
		{
			//>image.at<Vec3b>(i, j)[0]  y用float 崩溃
			cout << " " << R.at<double>(0,0)<< "  " << img_row_Points[n].x << " " << T.at<double>(0, 0);

			objectPoints[n].x = (float)(R.at<double>(0, 0) *  img_row_Points[n].x
			+ R.at<double>(0, 1) *  img_row_Points[n].y
			 + R.at<double>(0, 2) *  img_row_Points[n].z);
			objectPoints[n].y = R.at<float>(1, 0) *  img_row_Points[n].x
				+ R.at<double>(1, 1) *  img_row_Points[n].y
				+ R.at<double>(1, 2) *  img_row_Points[n].z;
			objectPoints[n].z = R.at<float>(2, 0) *  img_row_Points[n].x
				+ R.at<double>(2, 1) *  img_row_Points[n].y
				+ R.at<double>(2, 2) *  img_row_Points[n].z;


			objectPoints[n].x += T.at<double>(0, 0);
			objectPoints[n].y += T.at<double>(0, 1);
			objectPoints[n].z += T.at<double>(0, 2);
		}








	//打开投影仪在图片上画16个圆圈，投出去
	cv::Mat pro_img(cv::Size(1024, 768), CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < 16; i++) {
		
			cv::circle(pro_img, cv::Point2f(objectPoints[i].x ,
				objectPoints[i].y ), 10, CV_RGB(255, 0, 0), 3, 8, 0);
		}
	
	//在qt gui中不能用啊
	cv::namedWindow("pro", CV_WINDOW_AUTOSIZE);
	cv::imshow("pro", pro_img);
	cv::waitKey(30);
	cv::moveWindow("pro", 1024 , 0);
	cv::imwrite("pro_img_16.png", pro_img);

	//相机采集，在这张图上画定义的16个圆圈 看是否重合
	cv::waitKey(30);
	cv::VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	// while(true)
	cv::waitKey(30);
	cv::Mat cam_img(cv::Size(1920, 1080), CV_8UC3);
	cap >> cam_img;
	

	cv::namedWindow("cam", CV_WINDOW_AUTOSIZE);
	cv::imshow("cam", cam_img);
	cv::waitKey(30);
	for (int i = 0; i < 4; i++) {
		
			cv::circle(cam_img, cv::Point2f(img_row_Points[i].x ,
				img_row_Points[i].y), 10, CV_RGB(0, 0, 255), 3, 8, 0);
		}
	cv::imshow("cam", cam_img);
	cv::waitKey(30);
	cv::imwrite("cam_img_16.png", cam_img);



	return 0;
}

