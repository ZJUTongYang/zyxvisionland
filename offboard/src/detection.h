#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <time.h>

using namespace std;
using namespace cv;

typedef struct flag_detection flag_detection_t;
struct flag_detection
{
    //whether there is a flag
    bool result;

    //whether there is a flag in the past period
    bool prev_result;

    //image center
    Point img_center;

    //save the corner points of the flag
    vector< vector<Point> > corners;

    //save the area of the flag
    int img_area;

    //save the real area of the flag
    int square_area;

    //save the center point of the flag
    Point center;

    //save the previous center of the flag
    vector<Point> prev_centers;

    //save the distort yaml of the camera
    vector<float> distort;

    //save the 3D points of the real world
    vector<Point3f> points3d;

    //rotation matrix
    Mat R;

    //transition matrix
    Mat t;

    //real world position of camera center using solvePnP
    Point3d P_w;

    //flag position in camera axis using area
    Point3f Pc;

    //flag position in real world axis using area
    Point3d Pw;

    //flag center position in the camera axis
    //Mat P_c;

    vector<Point3f> fore_points;
    
    Point3f avg_point;
};

typedef struct Euler Euler_t;
struct Euler
{
    double roll;
    double pitch;
    double yaw;
};

flag_detection_t *flag_detection_create();

void flag_detection_destroy(flag_detection_t *td);

void flag_detection_clear(flag_detection_t *td);

void detect(Mat &frame, flag_detection_t *td);

void EstimatePose( flag_detection_t *td, Euler_t current_att);

int otsuThreshold(IplImage* img);

void FindSquares( IplImage* src, CvSeq* squares, CvMemStorage* storage, vector<Point> &squares_centers, vector< vector<Point> > &squares_v, Point pt );

void DrawSquares( IplImage* img, vector< vector<Point> > &squares_v , Point flag_center, const char* wndname);

int ComputeDistance( Point pt1, Point pt2 );

void CenterClassification( vector<Point> &squares_centers, vector< vector<int> > &result);

void Read3dPoints(vector<Point3f> &points3d);

void RankPoint( vector<Point> &p, int num = 4 );

void DrawTrajectory( Point3d P );

int ComputeArea( vector<Point> &vecPoly );

#endif