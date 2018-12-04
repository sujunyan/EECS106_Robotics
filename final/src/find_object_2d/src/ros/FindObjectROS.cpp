/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "FindObjectROS.h"

#include <std_msgs/Float32MultiArray.h>
#include "find_object_2d/ObjectsStamped.h"

#include <cmath>

using namespace find_object;

FindObjectROS::FindObjectROS(QObject *parent) : FindObject(true, parent),
												objFramePrefix_("object")
{
	ros::NodeHandle pnh("~"); // public
	pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);
	ROS_INFO("object_prefix = %s", objFramePrefix_.c_str());

	ros::NodeHandle nh; // public

	pub_ = nh.advertise<std_msgs::Float32MultiArray>("objects", 1);
	pubStamped_ = nh.advertise<find_object_2d::ObjectsStamped>("objectsStamped", 1);

	this->connect(this, SIGNAL(objectsFound(find_object::DetectionInfo)), this, SLOT(publish(find_object::DetectionInfo)));
}

void FindObjectROS::publish(const find_object::DetectionInfo &info)
{
	// send tf before the message
	if (info.objDetected_.size() && !depth_.empty() && depthConstant_ != 0.0f)
	{
		std::vector<tf::StampedTransform> transforms;
		char multiSubId = 'b';
		int previousId = -1;
		QMultiMap<int, QSize>::const_iterator iterSizes = info.objDetectedSizes_.constBegin();
		for (QMultiMap<int, QTransform>::const_iterator iter = info.objDetected_.constBegin();
			 iter != info.objDetected_.constEnd();
			 ++iter, ++iterSizes)
		{
			// get data
			int id = iter.key();
			float objectWidth = iterSizes->width();
			float objectHeight = iterSizes->height();

			QString multiSuffix;
			if (id == previousId)
			{
				multiSuffix = QString("_") + multiSubId++;
			}
			else
			{
				multiSubId = 'b';
			}
			previousId = id;

			// Find center of the object
			QPointF center = iter->map(QPointF(objectWidth / 2, objectHeight / 2));
			QPointF xAxis = iter->map(QPointF(3 * objectWidth / 4, objectHeight / 2));
			QPointF yAxis = iter->map(QPointF(objectWidth / 2, 3 * objectHeight / 4));

			cv::Vec3f center3D = this->getDepth(depth_,
												center.x() + 0.5f, center.y() + 0.5f,
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			cv::Vec3f axisEndX = this->getDepth(depth_,
												xAxis.x() + 0.5f, xAxis.y() + 0.5f,
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			cv::Vec3f axisEndY = this->getDepth(depth_,
												yAxis.x() + 0.5f, yAxis.y() + 0.5f,
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			if (std::isfinite(center3D.val[0]) && std::isfinite(center3D.val[1]) && std::isfinite(center3D.val[2]) &&
				std::isfinite(axisEndX.val[0]) && std::isfinite(axisEndX.val[1]) && std::isfinite(axisEndX.val[2]) &&
				std::isfinite(axisEndY.val[0]) && std::isfinite(axisEndY.val[1]) && std::isfinite(axisEndY.val[2]))
			{
				tf::StampedTransform transform;
				transform.setIdentity();
				transform.child_frame_id_ = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
				transform.frame_id_ = frameId_;
				transform.stamp_ = stamp_;
				transform.setOrigin(tf::Vector3(center3D.val[0], center3D.val[1], center3D.val[2]));

				//set rotation
				tf::Vector3 xAxis(axisEndX.val[0] - center3D.val[0], axisEndX.val[1] - center3D.val[1], axisEndX.val[2] - center3D.val[2]);
				xAxis.normalize();
				tf::Vector3 yAxis(axisEndY.val[0] - center3D.val[0], axisEndY.val[1] - center3D.val[1], axisEndY.val[2] - center3D.val[2]);
				yAxis.normalize();
				tf::Vector3 zAxis = xAxis.cross(yAxis);
				zAxis.normalize();
				tf::Matrix3x3 rotationMatrix(
					xAxis.x(), yAxis.x(), zAxis.x(),
					xAxis.y(), yAxis.y(), zAxis.y(),
					xAxis.z(), yAxis.z(), zAxis.z());
				tf::Quaternion q;
				rotationMatrix.getRotation(q);
				// set x axis going front of the object, with z up and z left
				q *= tf::createQuaternionFromRPY(CV_PI / 2.0, CV_PI / 2.0, 0);
				transform.setRotation(q.normalized());

				transforms.push_back(transform);
			}
			else
			{
				ROS_WARN("Object %d detected, center 2D at (%f,%f), but invalid depth, cannot set frame \"%s\"! "
						 "(maybe object is too near of the camera or bad depth image)\n",
						 id,
						 center.x(), center.y(),
						 QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString().c_str());
			}
		}
		if (transforms.size())
		{
			tfBroadcaster_.sendTransform(transforms);
		}
	}

	if (pub_.getNumSubscribers() || pubStamped_.getNumSubscribers())
	{
		std_msgs::Float32MultiArray msg;
		find_object_2d::ObjectsStamped msgStamped;
		msg.data = std::vector<float>(info.objDetected_.size() * 12);
		msgStamped.objects.data = std::vector<float>(info.objDetected_.size() * 12);
		int i = 0;
		QMultiMap<int, QSize>::const_iterator iterSizes = info.objDetectedSizes_.constBegin();
		for (QMultiMap<int, QTransform>::const_iterator iter = info.objDetected_.constBegin();
			 iter != info.objDetected_.constEnd();
			 ++iter, ++iterSizes)
		{
			msg.data[i] = msgStamped.objects.data[i] = iter.key();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iterSizes->width();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iterSizes->height();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m11();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m12();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m13();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m21();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m22();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m23();
			++i;
			msg.data[i] = msgStamped.objects.data[i] = iter->m31();
			++i; // dx
			msg.data[i] = msgStamped.objects.data[i] = iter->m32();
			++i; // dy
			msg.data[i] = msgStamped.objects.data[i] = iter->m33();
			++i;
		}
		if (pub_.getNumSubscribers())
		{
			pub_.publish(msg);
		}
		if (pubStamped_.getNumSubscribers())
		{
			// use same header as the input image (for synchronization and frame reference)
			msgStamped.header.frame_id = frameId_;
			msgStamped.header.stamp = stamp_;
			pubStamped_.publish(msgStamped);
		}
	}
}

void FindObjectROS::setDepthData(const std::string &frameId,
								 const ros::Time &stamp,
								 const cv::Mat &depth,
								 float depthConstant)
{
	frameId_ = frameId;
	stamp_ = stamp;
	depth_ = depth;
	depthConstant_ = depthConstant;
}

cv::Vec3f FindObjectROS::getDepth(const cv::Mat &depthImage,
								  int x, int y,
								  float cx, float cy,
								  float fx, float fy)
{
	if (!(x >= 0 && x < depthImage.cols && y >= 0 && y < depthImage.rows))
	{
		ROS_ERROR("Point must be inside the image (x=%d, y=%d), image size=(%d,%d)",
				  x, y,
				  depthImage.cols, depthImage.rows);
		return cv::Vec3f(
			std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::quiet_NaN());
	}

	cv::Vec3f pt;

	// Use correct principal point from calibration
	float center_x = cx; //cameraInfo.K.at(2)
	float center_y = cy; //cameraInfo.K.at(5)

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	float unit_scaling = isInMM ? 0.001f : 1.0f;
	float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
	float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	float depth;
	bool isValid;
	if (isInMM)
	{
		depth = (float)depthImage.at<uint16_t>(y, x);
		isValid = depth != 0.0f;
	}
	else
	{
		depth = depthImage.at<float>(y, x);
		isValid = std::isfinite(depth);
	}

	// Check for invalid measurements
	if (!isValid)
	{
		pt.val[0] = pt.val[1] = pt.val[2] = bad_point;
	}
	else
	{
		// Fill in XYZ
		pt.val[0] = (float(x) - center_x) * depth * constant_x;
		pt.val[1] = (float(y) - center_y) * depth * constant_y;
		pt.val[2] = depth * unit_scaling;
	}
	return pt;
}

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//added by zishu

float get_rand_bias(){
	return (1 + (rand() % 100 - 50) / 1000.0);
}

vector< vector<Point> > color_based_find_object(const cv::Mat &imgOriginal, int iLowH,  int iHighH, int iLowS, int iHighS, int iLowV, int iHighV, char* name){

	Mat imgHSV;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// cv::Mat element_9(9, 9, CV_8U, cv::Scalar(1));
	// cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element_9);

	Mat canny_output;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int thresh = 100;
	RNG rng(12345);

	/// Detect edges using canny
	Canny(imgThresholded, canny_output, thresh, thresh * 2, 3);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	extern bool image_show_open;
	if (image_show_open){
		imshow(name, imgThresholded); //show the thresholded image
	}
	return contours;
}


void FindObjectROS::detect(const cv::Mat &imgOriginal)
{

	extern int iLowH;
	extern int iHighH;
	extern int iLowS;
	extern int iHighS;
	extern int iLowV;
	extern int iHighV;

	extern int red_iLowH;
	extern int red_iHighH;
	extern int red_iLowS;
	extern int red_iHighS;
	extern int red_iLowV;
	extern int red_iHighV;
	extern bool image_show_open;
	vector< vector<Point> > green_contours;
	vector< vector<Point> > red_contours;
	int thresh = 100;
	RNG rng(12345);
	if (image_show_open){
		namedWindow("Control_green", CV_WINDOW_AUTOSIZE); //create a window called "Control_green"
		//Create trackbars in "Control_green" window by zishu
		cvCreateTrackbar("LowH", "Control_green", &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control_green", &iHighH, 179);
		cvCreateTrackbar("LowS", "Control_green", &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control_green", &iHighS, 255);
		cvCreateTrackbar("LowV", "Control_green", &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control_green", &iHighV, 255);

		namedWindow("Control_red", CV_WINDOW_AUTOSIZE); //create a window called "Control_red"
		//Create trackbars in "Control" window by zishu
		cvCreateTrackbar("LowH", "Control_red", &red_iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control_red", &red_iHighH, 179);
		cvCreateTrackbar("LowS", "Control_red", &red_iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control_red", &red_iHighS, 255);
		cvCreateTrackbar("LowV", "Control_red", &red_iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control_red", &red_iHighV, 255);
	}
	// Mat imgHSV;

	// cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	// Mat imgThresholded;

	// inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	// //morphological opening (remove small objects from the foreground)
	// erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// //morphological closing (fill small holes in the foreground)
	// dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// // cv::Mat element_9(9, 9, CV_8U, cv::Scalar(1));
	// // cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element_9);

	// Mat canny_output;
	// vector< vector<Point> > contours;
	// vector<Vec4i> hierarchy;
	// int thresh = 100;
	// RNG rng(12345);

	// /// Detect edges using canny
	// Canny(imgThresholded, canny_output, thresh, thresh * 2, 3);
	// /// Find contours
	// findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	green_contours = color_based_find_object(imgOriginal, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, "green");	
	red_contours = color_based_find_object(imgOriginal, red_iLowH, red_iHighH, red_iLowS, red_iHighS, red_iLowV, red_iHighV, "red");

	// int cmax = 0;
	// vector< vector<Point> > ::iterator itc = contours.begin();
	// while (itc != contours.end())
	// {
	// 	if (itc->size() > cmax)
	// 		cmax = itc->size();
	// 	itc++;
	// }

	// itc = contours.begin(); 
	// while (itc != contours.end())
	// {
	// 	if (itc->size() != cmax)
	// 		itc = contours.erase(itc);
	// 	else
	// 		itc++;
	// }

	int cmax = 0;
	vector< vector<Point> > ::iterator itc = green_contours.begin();
	while (itc != green_contours.end())
	{
		if (itc->size() > cmax)
			cmax = itc->size();
		itc++;
	}

	itc = green_contours.begin();
	int check = 0;
	while (itc != green_contours.end())
	{
		if (itc->size() != cmax || check == 1)
			itc = green_contours.erase(itc);
		else{
			itc++;
			check = 1;
		}		
	}

	vector< vector<Point> > green_contours_poly(green_contours.size());
	vector<Rect> green_boundRect(green_contours.size());
	vector<Point2f> green_center(green_contours.size());
	vector<float> green_radius(green_contours.size());
	if (green_contours.size() >0 ){

	//draw green rect on orignal image
	
	for (int i = 0; i < green_contours.size(); i++)
	{
		approxPolyDP(Mat(green_contours[i]), green_contours_poly[i], 3, true);
		green_boundRect[i] = boundingRect(Mat(green_contours_poly[i]));
		// minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}

	//draw red rect on orignal image
	vector< vector<Point> > red_contours_poly(red_contours.size());
	vector<Rect> red_boundRect(red_contours.size());
	vector<Point2f> red_center(red_contours.size());
	vector<float> red_radius(red_contours.size());
	for (int i = 0; i < red_contours.size(); i++)
	{
		approxPolyDP(Mat(red_contours[i]), red_contours_poly[i], 3, true);
		red_boundRect[i] = boundingRect(Mat(red_contours_poly[i]));
		// minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}

	int check_valid_parten = 0;
	
	float x_green = green_boundRect[0].x;
	float y_green = green_boundRect[0].y;
	float width_green = green_boundRect[0].width;
	float height_green = green_boundRect[0].height;

	vector<Rect> ::iterator i_rect = red_boundRect.begin();
	itc = red_contours.begin();
	while (i_rect != red_boundRect.end())
	{
		float x_red = i_rect->x;
		float y_red = i_rect->y;
		float width_red = i_rect->width;
		float height_red = i_rect->height;

		if (x_red > x_green && y_red > y_green && 
			(x_red + width_red < x_green + width_green) && 
			(y_red + height_red < y_green + height_green)){
			check_valid_parten = 1;
			i_rect++;
			itc++;
		}
			
		else{
			i_rect = red_boundRect.erase(i_rect);
			itc = red_contours.erase(itc);
		}
	}

	if (!check_valid_parten){
		
		green_contours.erase(green_contours.begin());
		green_boundRect.erase(green_boundRect.begin());

		i_rect = red_boundRect.begin();
		itc = red_contours.begin();
		while (i_rect != red_boundRect.end()){
			i_rect = red_boundRect.erase(i_rect);
			itc = red_contours.erase(itc);
		}
	}

	// /// Draw contours
	// Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	// for (int i = 0; i < contours.size(); i++)
	// {
	// 	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	// 	drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
	// 	rectangle(imgOriginal, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
	// }
	for (int i = 0; i < green_contours.size(); i++){
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		rectangle(imgOriginal, green_boundRect[i].tl(), green_boundRect[i].br(), color, 2, 8, 0);
	}
	for (int i = 0; i < red_contours.size(); i++){
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		rectangle(imgOriginal, red_boundRect[i].tl(), red_boundRect[i].br(), color, 2, 8, 0);
	}
	}
	if (image_show_open){
		imshow("Original", imgOriginal);			 //show the original image
	}
	/// Show in a window
	// namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	// imshow("Contours", drawing);

	if (green_contours.size() > 0)
	{
		ROS_INFO("send tranform start");
		std::vector<tf::StampedTransform> transforms;
		float x = green_boundRect[0].x;
		float y = green_boundRect[0].y;
		float width = green_boundRect[0].width;
		float height = green_boundRect[0].height;
		//get 3d information
		// Find center of the object
		int try_time = 10;
		while (try_time > 0)
		{
			
			try_time--;
			QPointF center = QPointF(x + width / 2 * get_rand_bias() , y + height / 2 * get_rand_bias());
			QPointF xAxis = QPointF(x + 5 * width / 8 * get_rand_bias(), y + height / 2 * get_rand_bias());
			QPointF yAxis = QPointF(x + width / 2 * get_rand_bias(), y + 5 * height / 8 * get_rand_bias());

			cv::Vec3f center3D = this->getDepth(depth_,
												round(center.x() + 0.5f), round(center.y() + 0.5f),
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			cv::Vec3f axisEndX = this->getDepth(depth_,
												round(xAxis.x() + 0.5f), round(xAxis.y() + 0.5f),
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			cv::Vec3f axisEndY = this->getDepth(depth_,
												round(yAxis.x() + 0.5f), round(yAxis.y() + 0.5f),
												float(depth_.cols / 2) - 0.5f, float(depth_.rows / 2) - 0.5f,
												1.0f / depthConstant_, 1.0f / depthConstant_);

			int id = 1; //debug
			QString multiSuffix = "green_1";
			if (std::isfinite(center3D.val[0]) && std::isfinite(center3D.val[1]) && std::isfinite(center3D.val[2]) &&
				std::isfinite(axisEndX.val[0]) && std::isfinite(axisEndX.val[1]) && std::isfinite(axisEndX.val[2]) &&
				std::isfinite(axisEndY.val[0]) && std::isfinite(axisEndY.val[1]) && std::isfinite(axisEndY.val[2]))
			{
				tf::StampedTransform transform;
				transform.setIdentity();
				transform.child_frame_id_ = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
				transform.frame_id_ = frameId_;
				transform.stamp_ = stamp_;
				transform.setOrigin(tf::Vector3(center3D.val[0], center3D.val[1], center3D.val[2]));

				//set rotation
				tf::Vector3 xAxis(axisEndX.val[0] - center3D.val[0], axisEndX.val[1] - center3D.val[1], axisEndX.val[2] - center3D.val[2]);
				xAxis.normalize();
				tf::Vector3 yAxis(axisEndY.val[0] - center3D.val[0], axisEndY.val[1] - center3D.val[1], axisEndY.val[2] - center3D.val[2]);
				yAxis.normalize();
				tf::Vector3 zAxis = xAxis.cross(yAxis);
				zAxis.normalize();
				tf::Matrix3x3 rotationMatrix(
					xAxis.x(), yAxis.x(), zAxis.x(),
					xAxis.y(), yAxis.y(), zAxis.y(),
					xAxis.z(), yAxis.z(), zAxis.z());
				tf::Quaternion q;
				rotationMatrix.getRotation(q);
				// set x axis going front of the object, with z up and z left
				q *= tf::createQuaternionFromRPY(CV_PI / 2.0, CV_PI / 2.0, 0);
				transform.setRotation(q.normalized());

				transforms.push_back(transform);
			}
			else
			{
				ROS_WARN("Object %d detected, center 2D at (%f,%f), but invalid depth, cannot set frame \"%s\"! "
						 "(maybe object is too near of the camera or bad depth image)\n",
						 id,
						 center.x(), center.y(),
						 QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString().c_str());
			}

			if (transforms.size())
			{
				ROS_INFO("send tranform");
				tfBroadcaster_.sendTransform(transforms);
				return;
			}
		}
	}
}
