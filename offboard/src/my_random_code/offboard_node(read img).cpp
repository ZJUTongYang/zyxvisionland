#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

cv::VideoWriter w_cap("re_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20.0, cv::Size(752, 480));

void image_callback(const sensor_msgs::Image &msg_img)  
{
	//cout << "123" << endl;
    Mat frame;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg_img, "bgr8");
    frame = cvPtr->image;
    imshow("123",frame);
    cv::waitKey(1);
    //w_cap.write(frame);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber img_sub = nh.subscribe
            ("MT9V034/image", 10, image_callback);    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

	cv::namedWindow("123",CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
