/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
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

#include "detection.h" 

#define FLY 0
#define HOVER 1
#define LAND 2
#define ATT_STABLE_THRESHOLD 1
#define ARRIVE_POINT_THRESHOLD 0.09
#define ARRIVE_FLAG_THRESHOLD 1000
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
int state;
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

    if( td->result == 0 )     //if the number of class members is smaller than MIN_SQUARE_NUM, there is no flag          
		{
			//cout << "no flag" << endl;
            td->prev_centers.push_back(cvPoint(-1, -1));
            if(td->prev_centers.size() > MAX_PREV_CENTERS_NUM)
                td->prev_centers.erase(td->prev_centers.begin());
			IplImage img_squares = IplImage(frame);
			cvShowImage( "Square Detection Demo", &img_squares);
			waitKey(1);
		}
		else
		{
            td->prev_centers.push_back(td->center);
            if(td->prev_centers.size() > MAX_PREV_CENTERS_NUM)
                td->prev_centers.erase(td->prev_centers.begin());
			cout << td->center << endl;
			const char* wndname = "Square Detection Demo";
			IplImage img_squares = IplImage(frame);
			DrawSquares( &img_squares, td->corners, td->center, wndname);
            //cout << "flag center: " << td->t << endl;
			//DrawTrajectory(td->P_w);
        	waitKey(1);
		}

    td->prev_result = 0;
    for(int i = 0; i < td->prev_centers.size(); i++)
    {
        if(td->prev_centers[i].x >= 0 && td->prev_centers[i].y >= 0)
        {
            td->prev_result = 1;
            break;
        }
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
    float threshold;
    //threshold = ARRIVE_FLAG_THRESHOLD * current_position.pose.position.z / 4;
    threshold = ARRIVE_FLAG_THRESHOLD;
    if(((td->center.x-td->img_center.x)*(td->center.x-td->img_center.x) + (td->center.y-td->img_center.y)*(td->center.y-td->img_center.y)) < threshold)
        return 1;
    else
        return 0;
}

Point3d ComputeLinearVelocity(Point setpoint, Point mypoint)
{
    Point3d velocity;
    //velocity.x = 0.003 * (setpoint.x - mypoint.x);
    //velocity.y = 0.0045 * (setpoint.y - mypoint.y);
    //velocity.z = 0;
    velocity.x = 0.003 * (setpoint.x - mypoint.x);
    velocity.y = 0.0045 * (setpoint.y - mypoint.y);
    velocity.z = 0;
    if(velocity.x > 0.8)
        velocity.x = 0.8;
    else if(velocity.x < -0.8)
        velocity.x = -0.8;
    if(velocity.y > 0.8)
        velocity.y = 0.8;
    else if(velocity.y < -0.8)
        velocity.y = -0.8;

    ROS_INFO("%d  %d",setpoint.x - mypoint.x, setpoint.y - mypoint.y);
    //ROS_INFO("%d  %lf  %lf",state,twist.twist.linear.x,twist.twist.linear.y);

    return velocity;
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
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    /*ros::Subscriber img_sub = nh.subscribe
            ("MT9V034/image", 10, image_callback);   */
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
    geometry_msgs::TwistStamped twist;

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

    double origin_x, origin_y, origin_z;
	origin_x = current_position.pose.position.x;
	origin_y = current_position.pose.position.y;
	origin_z = current_position.pose.position.z;
    ROS_INFO("%f   %f   %f oh yeah!",origin_x, origin_y, origin_z);
    pose.pose.position.x = origin_x + 0;
    pose.pose.position.y = origin_y + 0;
    pose.pose.position.z = origin_z + 5;


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
        if(ros::Time::now() - start_time > ros::Duration(5.0))
            break;
        local_pos_pub.publish(pose);
        ROS_INFO("Position Control");
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){

        arrive_flag = ArriveFlag();

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
        bool auto_land = 0;
        switch(state)
        {
            case FLY:
                if(td->result == 0)
                    if(td->prev_result == 0)
                        ROS_INFO("there is a mistake in detection!");
                    else
                    {
                        for(int i = td->prev_centers.size() - 1; i >= 0; i--)
                            if(td->prev_centers[i].x >= 0 && td->prev_centers[i].y >= 0)
                            {
                                td->center = td->prev_centers[i];
                                break;
                            }
                    }

                velocity = ComputeLinearVelocity(td->img_center, td->center);
                twist.twist.linear.x = velocity.y;
                twist.twist.linear.y = velocity.x;
                twist.twist.linear.z = velocity.z;
                /*twist.twist.linear.x = 1;
                twist.twist.linear.y = 0;
                twist.twist.linear.z = 0;*/
                local_vel_pub.publish(twist);
                break;

            case HOVER:
                pose.pose.position.x = current_position.pose.position.x;
                pose.pose.position.y = current_position.pose.position.y;
                pose.pose.position.z = origin_z+3;
                local_pos_pub.publish(pose);
                break;

            case LAND:
                if( current_position.pose.position.z > (origin_z+1) )
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
        ROS_INFO("%d  %lf  %lf",state,twist.twist.linear.x,twist.twist.linear.y);

        //local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
