// Copyright (C) 2021 zhaohaowu <zhaohaowu@csu.edu.cn>
#include <ros/ros.h>
#include "aruco_test/position.h"
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <time.h>
#include "cvdrawingutils.h"

clock_t start,end;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_dev");
	ros::NodeHandle n;

	std::string intrinsic, T_pub_topic;
	float TheMarkerSize;
	aruco::CameraParameters TheCameraParameters;
	aruco::MarkerDetector MDetector;
	std::vector<aruco::Marker> TheMarkers;
	std::map<uint32_t, aruco::MarkerPoseTracker> MTracker; 
	cv::Mat T_wm33, T_wm34, T_mc, T_cr, T_cm, T_wr;

	n.getParam("/aruco_dev/intrinsic", intrinsic);
	TheCameraParameters.readFromXMLFile(intrinsic);
	n.getParam("/aruco_dev/marker_size", TheMarkerSize);
	MDetector.setDictionary("ALL_DICTS");  
	MDetector.getParameters().detectEnclosedMarkers(0);
	n.getParam("/aruco_dev/T_pub_topic", T_pub_topic);
	ros::Publisher pub = n.advertise<aruco_test::position>(T_pub_topic,1000); 

	std::vector<float> Twm33;
	n.getParam("/aruco_dev/extrinsic/Twm33", Twm33);
	T_wm33 = (cv::Mat_<float>(4,4) <<
	Twm33[0],  Twm33[1],  Twm33[2],  Twm33[3],
	Twm33[4],  Twm33[5],  Twm33[6],  Twm33[7],
	Twm33[8],  Twm33[9],  Twm33[10], Twm33[11],
	Twm33[12], Twm33[13], Twm33[14], Twm33[15]);

	std::vector<float> Twm34;
	n.getParam("/aruco_dev/extrinsic/Twm34", Twm34);
	T_wm34 = (cv::Mat_<float>(4,4) <<
	Twm34[0],  Twm34[1],  Twm34[2],  Twm34[3],
	Twm34[4],  Twm34[5],  Twm34[6],  Twm34[7],
	Twm34[8],  Twm34[9],  Twm34[10], Twm34[11],
	Twm34[12], Twm34[13], Twm34[14], Twm34[15]);

	std::vector<float> Tcr;
	n.getParam("/aruco_dev/extrinsic/Tcr", Tcr);
	T_cr = (cv::Mat_<float>(4,4) <<
	Tcr[0],  Tcr[1],  Tcr[2],  Tcr[3],
	Tcr[4],  Tcr[5],  Tcr[6],  Tcr[7],
	Tcr[8],  Tcr[9],  Tcr[10], Tcr[11],
	Tcr[12], Tcr[13], Tcr[14], Tcr[15]);

	cv::VideoCapture videox;
	videox.open(0);
	cv::Mat img;
	while(ros::ok())
	{
		videox >> img;
		TheMarkers = MDetector.detect(img, TheCameraParameters, TheMarkerSize);
		if (TheCameraParameters.isValid() && TheMarkerSize > 0)
    	for (unsigned int i = 0; i < TheMarkers.size(); i++)
    	{
			TheMarkers[i].draw(img, cv::Scalar(0, 0, 255),2,true);
    		aruco::CvDrawingUtils::draw3dCube(img, TheMarkers[i], TheCameraParameters);
    		aruco::CvDrawingUtils::draw3dAxis(img, TheMarkers[i], TheCameraParameters);
    	}
		for (auto& marker : TheMarkers)
    	{
        	MTracker[marker.id].estimatePose(marker, TheCameraParameters, TheMarkerSize);
			//get marker from https://chev.me/arucogen/
        	T_cm = MTracker[marker.id].getRTMatrix();//二维码坐标系到相机坐标系的变换矩阵,即相机坐标系下二维码的位置和姿态
			if(T_cm.rows != 0 && T_cm.cols != 0 && marker.id==33)
			{
				T_mc = T_cm.inv();
				T_wr = T_wm33 * T_mc * T_cr;
			}
			else if(T_cm.rows != 0 && T_cm.cols != 0 && marker.id==34)
			{
				T_mc = T_cm.inv();
				T_wr = T_wm34 * T_mc * T_cr;
			}
			// std::cout << "T_wr:\n" << T_wr << std::endl;
			if(T_mc.rows!=0)
			{
				aruco_test::position msg;
				msg.x = T_mc.at<float>(0, 3);
				msg.y = T_mc.at<float>(1, 3);
				msg.z = T_mc.at<float>(2, 3);
				msg.x0 = T_wr.at<float>(0, 0);
				msg.x1 = T_wr.at<float>(0, 1);
				msg.x2 = T_wr.at<float>(0, 2);
				msg.x3 = T_wr.at<float>(0, 3);
				msg.x4 = T_wr.at<float>(1, 0);
				msg.x5 = T_wr.at<float>(1, 1);
				msg.x6 = T_wr.at<float>(1, 2);
				msg.x7 = T_wr.at<float>(1, 3);
				msg.x8 = T_wr.at<float>(2, 0);
				msg.x9 = T_wr.at<float>(2, 1);
				msg.x10 = T_wr.at<float>(2, 2);
				msg.x11 = T_wr.at<float>(2, 3);
				msg.x12 = T_wr.at<float>(3, 0);
				msg.x13 = T_wr.at<float>(3, 1);
				msg.x14 = T_wr.at<float>(3, 2);
				msg.x15 = T_wr.at<float>(3, 3);
				pub.publish(msg);
			}
		}
		cv::imshow("aruco_detect",img);
		cv::waitKey(1);	
	}
	ros::spin();
	return 0;
}
