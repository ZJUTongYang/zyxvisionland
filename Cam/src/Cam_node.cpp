#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termio.h>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> 

#include "../../CqUsbCam/CqUsbCam.h"
#include "../../CqUsbCam/SensorCapbablity.h"
//#include "/home/zhengyuxin/zhazha/Cam_ws/src/CqUsbCam/CqUsbCam.h"
//#include "/home/zhengyuxin/zhazha/Cam_ws/src/CqUsbCam/SensorCapbablity.h"

using namespace cv;
using namespace std;

string sensor = "MT9V034";
unsigned int g_width=752;
unsigned int g_height=480;
sensor_msgs::ImagePtr msg;
CCqUsbCam cam0;
int time1,time2;

void Disp(unsigned char* frameData)
{
	/*time1 = clock();
	printf("Frame Rate: %d/n",1000000/(time1 - time2));
	time2 = clock();*/
	cv::Mat frame(g_height, g_width, CV_8UC1, (unsigned char*)frameData);		
	msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();
	//imshow("disp",frame);
	//cv::waitKey(1);
}


int main(int argc, char *argv[])
{
  	ros::init(argc, argv, "Cam_node");
  	ros::NodeHandle nh;
	ros::Rate rate(20);

	image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("MT9V034/image", 1);

	cam0.SelectSensor(sensor);

	int usbCnt=CCqUsbCam::OpenUSB();
	printf("%d usb device(s) found!\n", usbCnt);
	if(usbCnt<=0)
	{
		printf("exiting ...\n");
		return -1;
	}
	cam0.ClaimInterface(0);
	cam0.StartCap(g_height,  g_width, Disp);

	while(nh.ok())
	{
		//cv::namedWindow("disp",CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
		pub.publish(msg);
		ros::spinOnce();
    	rate.sleep();
	}
	return 0;
}



