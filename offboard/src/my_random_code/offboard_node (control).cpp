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
#include <mavros_msgs/PositionTarget.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>
#include <math.h>

#include "detection.h" 

#define FLY 0
#define HOVER 1
#define LAND 2
#define ATT_STABLE_THRESHOLD 1
#define ARRIVE_POINT_THRESHOLD 0.09
#define ARRIVE_FLAG_THRESHOLD 10
#define MAX_FORE_POINTS_NUM 20
#define PI 3.1415926535898

using namespace cv;
using namespace std;

flag_detection_t *td;
bool arrive_flag;
bool arrive_point;
bool flag_exist;
bool att_stable;
bool collect_over;
int state;
int mode;
cv::VideoWriter w_cap("re_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20.0, cv::Size(752, 480));

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

        euler.yaw = -2 * sign * (double)atan2(q_x, q_w) / PI * 180; // yaw

        euler.pitch = sign * (PI / 2.0) / PI * 180; // pitch

        euler.roll = 0; // roll

    }
    else
    {
        euler.roll = atan2(2 * (q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z) / PI * 180;
        euler.pitch = asin(-2 * (q_x*q_z - q_w*q_y)) / PI * 180;
        euler.yaw = atan2(2 * (q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z) / PI * 180;
    }
        
    return euler;
}

bool Att_stable(const geometry_msgs::PoseStamped &pose)
{
    Euler_t current_att;
    current_att = QuaternionToEuler(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    if(current_att.roll <  ATT_STABLE_THRESHOLD && current_att.pitch <  ATT_STABLE_THRESHOLD)
        return 1;
    else
        return 0;
    //cout << current_att.roll << " || " << current_att.pitch << " || " << current_att.yaw << endl;
}

void image_callback(const sensor_msgs::Image &msg_img)  
{  
    Mat frame;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg_img, "bgr8");
    frame = cvPtr->image;
    //w_cap.write(frame);

    /*save IplImage
    IplImage* qImg;
    *qImg = IplImage(cvPtr->image);
    cvShowImage("down_camera",qImg);
    cvWaitKey(1);*/

    detect(frame, td);

    Point3f temp;
    if(arrive_point == 1 && att_stable == 1)
    {
        temp.x = td->t.at<double>(0,0);
        temp.y = td->t.at<double>(0,1);
        temp.z = td->t.at<double>(0,2);
        td->fore_points.push_back(temp);
        if(td->fore_points.size() > MAX_FORE_POINTS_NUM)
            td->fore_points.erase(td->fore_points.begin());
    }
    else
        td->fore_points.clear();

    //cout << td->fore_points << endl;
    td->avg_point.x = 0;
    td->avg_point.y = 0;
    td->avg_point.z = 0;
    collect_over = 0;
    if(td->fore_points.size() == MAX_FORE_POINTS_NUM)
    {
         for(int i = 0; i < MAX_FORE_POINTS_NUM; i++)
        {
            td->avg_point.x = td->avg_point.x + td->fore_points[i].x;
            td->avg_point.y = td->avg_point.y + td->fore_points[i].y;
            td->avg_point.z = td->avg_point.z + td->fore_points[i].z;            
        }
    collect_over = 1;
    }
    td->avg_point.x = td->avg_point.x / MAX_FORE_POINTS_NUM;
    td->avg_point.y = td->avg_point.y / MAX_FORE_POINTS_NUM;
    td->avg_point.z = td->avg_point.z / MAX_FORE_POINTS_NUM;

    //cout << td->avg_point << endl;  

    if( td->result == 0 )     //if the number of class members is smaller than MIN_SQUARE_NUM, there is no flag          
		{
			//cout << "no flag" << endl;
			IplImage img_squares = IplImage(frame);
			cvShowImage( "Square Detection Demo", &img_squares);
			waitKey(1);
		}
		else
		{
			//cout << td->center << endl;
			const char* wndname = "Square Detection Demo";
			IplImage img_squares = IplImage(frame);
			DrawSquares( &img_squares, td->corners, td->center, wndname);
            //cout << "flag center: " << td->t << endl;
			//DrawTrajectory(td->P_w);
        	waitKey(1);
		}
		flag_detection_clear(td);
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

bool ArriveFlag()
{
    if(arrive_point == 1 
        && fabs(td->avg_point.x) < ARRIVE_FLAG_THRESHOLD && fabs(td->avg_point.x) > 0
        && fabs(td->avg_point.y) < ARRIVE_FLAG_THRESHOLD && fabs(td->avg_point.y) > 0)
        return 1;
    else
        return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //initialize detection
    td = flag_detection_create();

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher local_thr_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    /*ros::Subscriber img_sub = nh.subscribe
            ("MT9V034/image", 10, image_callback);    */
    ros::Subscriber img_sub = nh.subscribe
            ("iris_down_cam/image_raw", 10, image_callback);    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1;
    pose.pose.position.y = -1;
    pose.pose.position.z = 5;
    /*pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;*/

    /*geometry_msgs::PoseStamped att;
    att.pose.position.x = 0;
    att.pose.position.y = 0;
    att.pose.position.z = 0;
    att.pose.orientation.x = 0;
    att.pose.orientation.y = 0;
    att.pose.orientation.z = 1;
    att.pose.orientation.w = 0;*/

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode offb_set_mode1;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode1.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

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

    while(ros::ok()){
        arrive_point = ArrivePoint(pose);
        flag_exist = FlagExist();
        arrive_flag = ArriveFlag();
        att_stable = Att_stable(current_position);

        int pstate = state;
        if(arrive_point == 0 || att_stable == 0)
            state = FLY;
        else if(flag_exist == 0)
                state = HOVER;
             else if(arrive_flag == 0)
                    {
                        if(collect_over == 1)
                            state = FLY;
                        else
                            state = HOVER;
                    }
                  else
                    state = LAND;

        if(pstate - state != 0)
        {
        switch(state)
        {
            case FLY:
                pose.pose.position.x = current_position.pose.position.x - td->avg_point.y/100;
                pose.pose.position.y = current_position.pose.position.y - td->avg_point.x/100;
                pose.pose.position.z = 3;
                break;

            case HOVER:
                pose.pose.position.x = current_position.pose.position.x;
                pose.pose.position.y = current_position.pose.position.y;
                pose.pose.position.z = current_position.pose.position.z;
                break;

            case LAND:
                //pose.pose.position.z = 0;
                /*if( current_state.mode != "AUTO.LAND" &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( set_mode_client.call(offb_set_mode1) &&
                            offb_set_mode1.response.mode_sent){        //(indigo)mode_sent-->success
                            ROS_INFO("autoland enabled");
                        }
                    last_request = ros::Time::now();
                }*/
                break;

            default:
                ROS_INFO("Unexpected Error!");
        }
        }

        if(state == LAND){
                if( current_state.mode != "AUTO.LAND" &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( set_mode_client.call(offb_set_mode1) &&
                            offb_set_mode1.response.mode_sent){        //(indigo)mode_sent-->success
                            ROS_INFO("autoland enabled");
                        }
                    last_request = ros::Time::now();
                }
        }

        ROS_INFO("%d  %d  %d  %d",arrive_point,flag_exist,arrive_flag,state);

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
