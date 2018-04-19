/**
 * 2018.4.7
 * This code is control based on image area
 * If you want to run this code on odroid, you must modify several places
 * First, we must subscribe camera data rather than gazebo camera data
 * Second, we must using manul offboard rather than auto offboard
 * Third, we must remove the image show
 * Forth, we must modify camera parameter
 * By Zhengyuxin
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <time.h>

#include "detection.h" 

#define FLY 0
#define HOVER 1
#define LAND 2
#define ATT_STABLE_THRESHOLD 1
#define ARRIVE_POINT_THRESHOLD 0.09
#define ARRIVE_FLAG_THRESHOLD 20
#define MAX_FORE_POINTS_NUM 20
#define MAX_PREV_CENTERS_NUM 50
#define PI 3.1415926535898

using namespace cv;
using namespace std;

flag_detection_t *td;
bool arrive_flag;
bool arrive_point;
bool flag_exist;
bool att_stable;
bool collect_over;
bool auto_land = 0;
int state;
int mode;
cv::VideoWriter w_cap("re_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20.0, cv::Size(752, 480));

ofstream f4("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp4.txt");
ofstream f5("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp5.txt");
ofstream f6("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp6.txt");

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
    //ROS_INFO("%f",current_position.pose.position.z);
    //ROS_INFO("ljslovzyx");
}

Euler_t QuaternionToEuler(double q_x, double q_y, double q_z, double q_w)
{
    //double detect_begin = clock();
    Euler_t euler;
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;

    double TEST = q_w * q_y - q_x * q_z;

    if (TEST < -Threshold || TEST > Threshold)
    {
        int sign;

        if(TEST > 0)
            sign = 1;
            else if(TEST < 0)
                    sign = -1;
                else
                    sign = 0;

        //euler.yaw = -2 * sign * (double)atan2(q_x, q_w) / PI * 180; // yaw
        euler.yaw = -2 * sign * (double)atan2(q_x, q_w);

        //euler.pitch = sign * (PI / 2.0) / PI * 180; // pitch
        euler.pitch = sign * (PI / 2.0);

        euler.roll = 0; // roll

    }
    else
    {
        /*euler.roll = atan2(2 * (q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z) / PI * 180;
        euler.pitch = asin(-2 * (q_x*q_z - q_w*q_y)) / PI * 180;
        euler.yaw = atan2(2 * (q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z) / PI * 180;*/
        euler.roll = atan2(2 * (q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z);
        euler.pitch = asin(-2 * (q_x*q_z - q_w*q_y));
        euler.yaw = atan2(2 * (q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z);
    }

    //cout << "QuaternionToEuler Time is " << ( clock() - detect_begin ) / CLOCKS_PER_SEC * 1000 << endl;    
    return euler;
}

bool Att_stable(const geometry_msgs::PoseStamped &pose)
{
    Euler_t current_att;
    current_att = QuaternionToEuler(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    if(current_att.roll * PI / 180 <  ATT_STABLE_THRESHOLD && current_att.pitch * PI / 180 <  ATT_STABLE_THRESHOLD)
        return 1;
    else
        return 0;
    //cout << current_att.roll << " || " << current_att.pitch << " || " << current_att.yaw << endl;
}

void image_callback(const sensor_msgs::Image &msg_img)  
{
    ros::Time detect_begin = ros::Time::now();
    //double detect_begin = clock()
    Mat frame;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg_img, "bgr8");
    frame = cvPtr->image;
    //w_cap.write(frame);
    ros::Duration d1(ros::Time::now() - detect_begin);
    std::cout << "ImageCopy Time is " << d1*1000<<std::endl;

    /*save IplImage
    IplImage* qImg;
    *qImg = IplImage(cvPtr->image);
    cvShowImage("down_camera",qImg);
    cvWaitKey(1);*/

    detect(frame, td); 

    if( td->result == 0 )     //if the number of class members is smaller than MIN_SQUARE_NUM, there is no flag          
		{
			//cout << "no flag" << endl;
			IplImage img_squares = IplImage(frame);
			cvShowImage( "Square Detection Demo", &img_squares);
			waitKey(1);
		}
		else
		{
            Euler_t current_att;
            current_att = QuaternionToEuler(current_position.pose.orientation.x, current_position.pose.orientation.y, current_position.pose.orientation.z, current_position.pose.orientation.w);
            EstimatePose(td, current_att);
            //cout << td->P_w << endl;
            f4 << current_position.pose.position.x << endl;
	        f5 << current_position.pose.position.y << endl;
	        f6 << current_position.pose.position.z << endl;
			//cout << td->center << endl;
			const char* wndname = "Square Detection Demo";
			IplImage img_squares = IplImage(frame);
            vector< vector<Point> > zyx;
            //zyx.push_back(td->corners[td->corners.size()-1]);
			DrawSquares( &img_squares, td->corners, td->center, wndname);
            //cout << "flag center: " << td->t << endl;
			//DrawTrajectory(td->Pw);
        	waitKey(1);
		}
	flag_detection_clear(td);
    ros::Time detect_finish = ros::Time::now();
    ros::Duration d(detect_finish - detect_begin);
    std::cout << "Image Callback Time is " << d*1000<<std::endl;
    //double frequency =CLOCKS_PER_SEC/(clock() - detect_begin);
    //ROS_INFO("Detection frequency: %lf", frequency);
}

bool ArrivePoint(const geometry_msgs::PoseStamped &pose)
{
    double dx, dy, dz, dd;
    dx = pose.pose.position.x - current_position.pose.position.x;
    dy = pose.pose.position.y - current_position.pose.position.y;
    dz = pose.pose.position.z - current_position.pose.position.z;
    dd = dx * dx + dy * dy + dz * dz;
    if(dd < ARRIVE_POINT_THRESHOLD)
        return 1;
    else
        return 0;
}

bool FlagExist()
{
    if(td->result == 0)
        return 0;
    else
        return 1;
}

bool ArriveFlag(Point3d Pw)
{
    if( fabs(Pw.x) < ARRIVE_FLAG_THRESHOLD && fabs(Pw.y) < ARRIVE_FLAG_THRESHOLD )
        return 1;
    else
        return 0;
}

Point3d ComputeLinearVelocity(Point3d Pw)
{
    Point3d velocity;
    velocity.x = -0.005 * Pw.y;
    velocity.y = 0.005 * Pw.x;
    velocity.z = 0;
    if(velocity.x > 0.8)
        velocity.x = 0.8;
    else if(velocity.x < -0.8)
        velocity.x = -0.8;
    if(velocity.y > 0.8)
        velocity.y = 0.8;
    else if(velocity.y < -0.8)
        velocity.y = -0.8;

    return velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //initialize detection
    td = flag_detection_create();

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, &position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber img_sub = nh.subscribe
           ("MT9V034/image", 1, image_callback);   
    //ros::Subscriber img_sub = nh.subscribe
    //        ("iris_down_cam/image_raw", 1, image_callback);    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;

    double origin_x = 0, origin_y = 0, origin_z = 0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        origin_x = origin_x + current_position.pose.position.x;
	    origin_y = origin_y + current_position.pose.position.y;
	    origin_z = origin_z + current_position.pose.position.z;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    origin_x = origin_x / 100;
    origin_y = origin_y / 100;
    origin_z = origin_z / 100;
    ROS_INFO("Origin Position: %f   %f   %f",origin_x, origin_y, origin_z);

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode offb_set_mode1;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode1.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    pose.pose.position.x = origin_x + 0;
    pose.pose.position.y = origin_y + 0;
    pose.pose.position.z = origin_z + 10;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        bool arm_flag=0;
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
                    arm_flag = 1;
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        if(arm_flag ==1)
            break;
    }

    ros::Time start_time = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() - start_time > ros::Duration(2.0))
            break;
        /*if(ros::Time::now() - start_time > ros::Duration(10.0))
        {
            pose.pose.position.x = 3;
            pose.pose.position.y = 0;
            pose.pose.position.z = 10;
        }
        if(ros::Time::now() - start_time > ros::Duration(20.0))
        {
            pose.pose.position.x = 3;
            pose.pose.position.y = 3;
            pose.pose.position.z = 10;
        }
        if(ros::Time::now() - start_time > ros::Duration(30.0))
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 3;
            pose.pose.position.z = 10;
        }
        if(ros::Time::now() - start_time > ros::Duration(40.0))
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 10;
        }*/
        local_pos_pub.publish(pose);
        ROS_INFO("Position Control");
        ros::spinOnce();
        rate.sleep();
    }

    /*while(ros::ok()){
        att_stable = Att_stable(current_position);
        arrive_point = ArrivePoint(pose);
        if(arrive_point == 1 && att_stable == 1)
        {
            if(td->Pw.z < 410)
                break;
            pose.pose.position.x = current_position.pose.position.x - td->Pw.y/100;
            pose.pose.position.y = current_position.pose.position.y + td->Pw.x/100;
            pose.pose.position.z = 4;
        }
        ROS_INFO("Camera Center: %lf %lf %lf",td->Pw.x, td->Pw.y, td->Pw.z);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }*/

    while(ros::ok()){
        ros::Time t0 = ros::Time::now();

        ros::spinOnce();

        att_stable = Att_stable(current_position);
        arrive_flag = ArriveFlag(td->Pw);

        if(td->result == 0)
            if(td->prev_result == 0)
                state = HOVER;
            else if( arrive_flag == 1 )
                    state = LAND;
                 else
                    state = FLY;
        else if( arrive_flag == 1 )
                    state = LAND;
                 else
                    state = FLY;
            

        Point3d velocity;
        switch(state)
        {
            case FLY:
                    velocity = ComputeLinearVelocity(td->Pw);
                    twist.twist.linear.x = velocity.x;
                    twist.twist.linear.y = velocity.y;
                    twist.twist.linear.z = 0;
                    twist.twist.angular.x = 0;
                    twist.twist.angular.y = 0;
                    twist.twist.angular.z = 0;
                    local_vel_pub.publish(twist);
                break;

            case HOVER:
                twist.twist.linear.x = 0;
                twist.twist.linear.y = 0;
                twist.twist.linear.z = 0;
                local_vel_pub.publish(twist);
                /*pose.pose.position.x = current_position.pose.position.x;
                pose.pose.position.y = current_position.pose.position.y;
                pose.pose.position.z = origin_z+3;
                local_pos_pub.publish(pose);*/
                break;

            case LAND:
                if( td->Pw.z > 150 )
                {
                    twist.twist.linear.x = 0;
                    twist.twist.linear.y = 0;
                    twist.twist.linear.z = -1;
                    local_vel_pub.publish(twist);
                }
                else
                    auto_land = 1;
                break;

            default:
                ROS_INFO("Unexpected Error!");
        }

        if(state == LAND && auto_land == 1){
                if( current_state.mode != "AUTO.LAND" &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( set_mode_client.call(offb_set_mode1) &&
                            offb_set_mode1.response.mode_sent){        //(indigo)mode_sent-->success
                            ROS_INFO("autoland enabled");
                        }
                    last_request = ros::Time::now();
                }
        }

        //ROS_INFO("%d  %d  %d  %d",arrive_point,flag_exist,arrive_flag,state);
        ROS_INFO("%d %lf %lf %lf",state, twist.twist.linear.x,twist.twist.linear.y,td->Pw.z);

        //local_pos_pub.publish(pose);
        ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);
        //std::cout << "YT: time is " << d*1000<<std::endl;
        //ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
