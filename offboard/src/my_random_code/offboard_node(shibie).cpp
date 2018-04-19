/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>

#include "detection.h" 

using namespace cv;

flag_detection_t *td; 

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void image_callback(const sensor_msgs::Image &msg_img)  
{  
    Mat frame;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg_img, "bgr8");
    frame = cvPtr->image;
    //imshow("down_camera",mImg);
    //cvWaitKey(1);

    /*
    IplImage* qImg;
    *qImg = IplImage(cvPtr->image);
    cvShowImage("down_camera",qImg);
    cvWaitKey(1);*/

    detect(frame, td);

    if( td->result == 0 )     //if the number of class members is smaller than MIN_SQUARE_NUM, there is no flag          
		{
			cout << "no flag" << endl;
			IplImage img_squares = IplImage(frame);
			cvShowImage( "Square Detection Demo", &img_squares);
			waitKey(1);
		}
		else
		{
			cout << td->center << endl;
			const char* wndname = "Square Detection Demo";
			IplImage img_squares = IplImage(frame);
			DrawSquares( &img_squares, td->corners, td->center, wndname);
			//DrawTrajectory(td->P_w);
        	waitKey(1);
		}
		flag_detection_clear(td);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //initialize detection
    td = flag_detection_create();
    cout << "1" << endl;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher local_thr_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber img_sub = nh.subscribe
            ("MT9V034/image", 10, image_callback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 1;
    pose.pose.position.z = 3;
    /*pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;*/

    geometry_msgs::PoseStamped att;
    att.pose.position.x = 0;
    att.pose.position.y = 0;
    att.pose.position.z = 0;
    att.pose.orientation.x = 0;
    att.pose.orientation.y = 0;
    att.pose.orientation.z = 1;
    att.pose.orientation.w = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){        //(indigo)mode_sent-->success
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
       // local_att_pub.publish(att);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
