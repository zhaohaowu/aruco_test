// Copyright (C) 2021 zhaohaowu <zhaohaowu@csu.edu.cn>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "aruco_test/position.h"
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <time.h>
#include "cvdrawingutils.h"

image_transport::Publisher img_pub;
ros::Subscriber sub;
ros::Publisher pub;
aruco::MarkerDetector MDetector;
cv::VideoCapture TheVideoCapturer;
std::vector<aruco::Marker> TheMarkers;
cv::Mat TheInputImageCopy;
aruco::CameraParameters TheCameraParameters;
float TheMarkerSize;
std::map<uint32_t, aruco::MarkerPoseTracker> MTracker; 

clock_t start,end;

class test
{
	public:
		// ros::NodeHandle n_;
		// test(ros::NodeHandle n):n_(n){}
		cv::Mat addImage(cv::Mat img); 
		cv::Mat T_cm, T_wr, T_wm33, T_wm34, T_mc, T_cr;
		cv::Mat pose;
};

cv::Mat test::addImage ( cv::Mat TheInputImageCopy )
{
    TheMarkers = MDetector.detect(TheInputImageCopy, TheCameraParameters, TheMarkerSize);
	if (TheCameraParameters.isValid() && TheMarkerSize > 0)
    for (unsigned int i = 0; i < TheMarkers.size(); i++)
    {
	TheMarkers[i].draw(TheInputImageCopy, cv::Scalar(0, 0, 255),2,true);
    aruco::CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
    aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
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
	// std::cout << "\n\n OK \n\n";
	cv::imshow("aruco_detect",TheInputImageCopy);
	cv::waitKey(1);	
	return TheInputImageCopy;
}

test* g_slam;

void callback(const sensor_msgs::CompressedImageConstPtr& img_ptr)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
	cv::Mat img = g_slam->addImage(cv_ptr->image);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	img_pub.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_test");
	ros::NodeHandle n;

	std::string intrinsic, image_sub_topic, image_pub_topic, T_pub_topic;
	n.getParam("/aruco_test/intrinsic", intrinsic);
	TheCameraParameters.readFromXMLFile(intrinsic);
	n.getParam("/aruco_test/marker_size", TheMarkerSize);
	MDetector.setDictionary("ALL_DICTS");  
	MDetector.getParameters().detectEnclosedMarkers(0); 
	// g_slam = new test(n);
	g_slam = new test();

	std::vector<float> Twm33;
	n.getParam("/aruco_test/extrinsic/Twm33", Twm33);
	g_slam->T_wm33 = (cv::Mat_<float>(4,4) <<
	Twm33[0],  Twm33[1],  Twm33[2],  Twm33[3],
	Twm33[4],  Twm33[5],  Twm33[6],  Twm33[7],
	Twm33[8],  Twm33[9],  Twm33[10], Twm33[11],
	Twm33[12], Twm33[13], Twm33[14], Twm33[15]);
	std::vector<float> Twm34;
	n.getParam("/aruco_test/extrinsic/Twm34", Twm34);
	g_slam->T_wm34 = (cv::Mat_<float>(4,4) <<
	Twm34[0],  Twm34[1],  Twm34[2],  Twm34[3],
	Twm34[4],  Twm34[5],  Twm34[6],  Twm34[7],
	Twm34[8],  Twm34[9],  Twm34[10], Twm34[11],
	Twm34[12], Twm34[13], Twm34[14], Twm34[15]);
	std::vector<float> Tcr;
	n.getParam("/aruco_test/extrinsic/Tcr", Tcr);
	g_slam->T_cr = (cv::Mat_<float>(4,4) <<
	Tcr[0],  Tcr[1],  Tcr[2],  Tcr[3],
	Tcr[4],  Tcr[5],  Tcr[6],  Tcr[7],
	Tcr[8],  Tcr[9],  Tcr[10], Tcr[11],
	Tcr[12], Tcr[13], Tcr[14], Tcr[15]);

	n.getParam("/aruco_test/image_pub_topic", image_pub_topic);
	n.getParam("/aruco_test/image_sub_topic", image_sub_topic);
	n.getParam("/aruco_test/T_pub_topic", T_pub_topic);
	sub = n.subscribe(image_sub_topic, 1000, callback);
	image_transport::ImageTransport it(n);
	img_pub = it.advertise(image_pub_topic, 1000);
	pub = n.advertise<aruco_test::position>(T_pub_topic,1000);

	ros::spin();
	delete g_slam;
	return 0;
}
