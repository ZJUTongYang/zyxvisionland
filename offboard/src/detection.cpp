#include "detection.h"

#define PI 3.1415926535879
#define IMG_AREA 1228800
#define MIN_SQUARE_AREA 100
#define MIN_CENTER_DIS 10
#define MIN_SQUARE_NUM 3

cv::Mat K( cv::Matx33f(315.5, 0, 376.5,
						 0, 315.5, 240.5,
						 0, 0, 1) );

/*cv::Mat K( cv::Matx33f(567.92, 0, 396.94,
						 0, 567.46, 240.63,
						 0, 0, 1) );*/

Point Oc;
ofstream f1("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp1.txt");
ofstream f2("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp2.txt");
ofstream f3("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp3.txt");
ofstream f7("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp7.txt");
ofstream f8("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp8.txt");
ofstream f9("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/temp9.txt");

int otsuThreshold(IplImage* img)
{
	
	int T = 0;
	int height = img->height;
	int width  = img->width;
	int step      = img->widthStep;
	int channels  = img->nChannels;
	uchar* data  = (uchar*)img->imageData;
	double gSum0;
	double gSum1;
	double N0 = 0;
	double N1 = 0;
	double u0 = 0;
	double u1 = 0;
	double w0 = 0;
	double w1 = 0;
	double u = 0;
	double tempg = -1;
	double g = -1;
	double Histogram[256]={0};// = new double[256];
	double N = width*height;
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			double temp =data[i*step + j * 3] * 0.114 + data[i*step + j * 3+1] * 0.587 + data[i*step + j * 3+2] * 0.299;
			temp = temp<0? 0:temp;
			temp = temp>255? 255:temp;
			Histogram[(int)temp]++;
		} 
	}
	
	for (int i = 0;i<256;i++)
	{
		gSum0 = 0;
		gSum1 = 0;
		N0 += Histogram[i];			
		N1 = N-N0;
		if(0==N1)break;
		w0 = N0/N;
		w1 = 1-w0;
		for (int j = 0;j<=i;j++)
		{
			gSum0 += j*Histogram[j];
		}
		u0 = gSum0/N0;
		for(int k = i+1;k<256;k++)
		{
			gSum1 += k*Histogram[k];
		}
		u1 = gSum1/N1;
		//u = w0*u0 + w1*u1;
		g = w0*w1*(u0-u1)*(u0-u1);
		if (tempg<g)
		{
			tempg = g;
			T = i;
		}
	}
	return T; 
}

void FindSquares( IplImage* src, CvSeq* squares, CvMemStorage* storage, vector<Point> &squares_centers, vector< vector<Point> > &squares_v, Point pt )
{
	CvSeq* cv_contours;    // 边缘
	CvSeq* result;         // the result of detecting squares
	CvSeqReader reader;    // the pointer to read data of "result"
	CvPoint corner[4];
	vector<Point> corner_v;
	Point temp;
	Point center;
	cvFindContours( src, storage, &cv_contours, sizeof(CvContour),CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, pt );
	while(cv_contours)
	{
		if( fabs(cvContourArea(cv_contours)) > MIN_SQUARE_AREA )   //neglect the small contours
		{
			result = cvApproxPoly( cv_contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(cv_contours)*0.02, 0 );
			if( result->total == 4  &&  cvCheckContourConvexity(result) )  
            {
				cvStartReadSeq( result, &reader, 0 );                      
                for( int i = 0; i < 4; i++ )
				{
					cvSeqPush( squares,(CvPoint*)cvGetSeqElem( result, i ));
					memcpy( corner + i, reader.ptr, result->elem_size ); 
        			CV_NEXT_SEQ_ELEM( result->elem_size, reader );
				}
				for(int i =0; i < 4; i++)    //save the corner points to corner_v, it will help us process the data
				{
					temp = corner[i];
					corner_v.push_back(temp);
				}
				center.x = (corner[0].x + corner[1].x + corner[2].x + corner[3].x) / 4;
				center.y = (corner[0].y + corner[1].y + corner[2].y + corner[3].y) / 4;
				squares_centers.push_back(center);   
				squares_v.push_back(corner_v);
				corner_v.clear();       
            } 
		}                                     
        cv_contours = cv_contours->h_next;    
	}
}

void DrawSquares( IplImage* img, vector< vector<Point> > &squares_v , Point flag_center, const char* wndname)
{   
    CvSeqReader reader;   
    IplImage* cpy = cvCloneImage( img );   
    CvPoint pt[4];           
    for( int i = 0; i < squares_v.size(); i++ )  
    {       
        CvPoint* rect = pt;    
        int count = 4;
		pt[0] = squares_v[i][0];
		pt[1] = squares_v[i][1]; 
		pt[2] = squares_v[i][2]; 
		pt[3] = squares_v[i][3];          
        //cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );
        //cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(rand()&255,rand()&255,rand()&255), 2, CV_AA, 0 );//彩色绘制
		cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 1, CV_AA, 0 );//彩色绘制
    }
	cvCircle( cpy, flag_center, 5, CV_RGB(255, 0, 0), 1);
	cvCircle( cpy, Oc, 2, CV_RGB(0, 0, 255), 1);
	cvCircle( cpy, cvPoint(376,240), 2, CV_RGB(0, 255, 255), 1);
    cvShowImage( wndname, cpy );  
    cvReleaseImage( &cpy );
}

int ComputeDistance( Point pt1, Point pt2 )
{
	int dsx = abs(pt1.x - pt2.x);
	int dsy = abs(pt1.y - pt2.y);
	return sqrt( dsx*dsx + dsy*dsy );
}

void CenterClassification( vector<Point> &squares_centers, vector< vector<int> > &result)
{
	vector<int> centers_index;
	vector<int> result_temp;
	int index_i;
	int index_j;
	for(int i = 0; i < squares_centers.size(); i++)
	{
		centers_index.push_back(i);    //save the index of squares centers
	}

	for(int i = 0; i < centers_index.size(); i++)
		{
			result_temp.push_back(centers_index[i]);
			for(int j = i + 1; j < centers_index.size(); j++)
			{
				index_i = centers_index[i];
				index_j = centers_index[j];
				if( ComputeDistance( squares_centers[index_i], squares_centers[index_j] ) < MIN_CENTER_DIS )
				{
					result_temp.push_back(centers_index[j]);
					centers_index.erase(centers_index.begin() + j - 1);
					j--;
				}
			}
			result.push_back(result_temp);
			result_temp.clear();
		}
}

void Read3dPoints(vector<Point3f> &points3d)
{
	Point3f temp3d;
	std::ifstream reader("/home/zhengyuxin/zhazha/Cam_ws/src/offboard/src/axis.txt");
	//std::ifstream reader("./axis.txt");

	if (!reader)
	{
		std::cout << "can't open"<< endl;
		system("pause");
		exit(0);
	}

	while (reader >> temp3d.x)
	{
		reader >> temp3d.y;
		reader >> temp3d.z;
		points3d.push_back(temp3d);
	}
	reader.close();
}

void RankPoint( vector<Point> &p, int num )
{
	vector<double> cosr;
	vector<Point> result;
	double ab, abs_a, abs_b;   //vector b is (1, 1) 
	int temp2 = p[0].x - p[0].y; 
	int right_up = 0;
	//find out the right_up point as first point
	for( int i = 1; i < num; i++)
	{
		if( p[i].x - p[i].y >= temp2 )
		{
			temp2 = p[i].x - p[i].y;
			right_up = i;
		}
	}
	result.push_back(p[right_up]);
	p.erase( p.begin() + right_up );

	//compute the angel between vector a and (1,1)
	for( int i = 0; i < num - 1; i++ )
	{
		ab = p[i].x - result[0].x + p[i].y - result[0].y;
		abs_a = sqrt((p[i].x - result[0].x) * (p[i].x - result[0].x) + (p[i].y - result[0].y) * (p[i].y - result[0].y));
		abs_b = sqrt(2);
		cosr.push_back(ab / abs_a /abs_b);
	}

	//sort the points' angels from big to small
	int index[num-1];
	int temp;
	for(int i = 0; i < num - 1; i++)
		index[i] = i;

	for(int i = 0; i < num -1; i++)
	{
		for(int j = i; j < num - 1; j++)
		{
			if(cosr[index[i]] < cosr[index[j]])
			{
				temp = index[i];
				index[i] = index[j];
				index[j] = temp;
			}
		}
	}

	//the points are clockwise now
	for(int i = 0; i < num - 1; i++ )
		result.push_back(p[index[i]]);
	
	swap(p,result);
}

void DrawTrajectory( Point3d P )
{
	IplImage* img = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
	for(int i = 0; i < img->width; i++)
		for(int j = 0; j < img->height; j++)
			((uchar*)(img->imageData + img->widthStep * j))[i] = (char)255;
	for(int i = 0; i < img->width; i++)
		((uchar*)(img->imageData + img->widthStep * (img->height/2)))[i] = (char)0;
	for(int j = 0; j < img->width; j++)
		((uchar*)(img->imageData + img->widthStep * j))[img->width/2] = (char)0;
	int x = P.x + img->width/2;
	int y = -P.y + img->height/2;
	cvCircle( img, CvPoint(x,y), 5, 0, 1);
	cvShowImage( "trajectory", img );  
    cvReleaseImage( &img );
}

//the points must be clockwise
int ComputeArea( vector<Point> &vecPoly )
{
	int iCycle,iCount,iArea;
    iCycle=0;
    iArea=0;
    iCount=vecPoly.size();

    for(iCycle=0;iCycle<iCount;iCycle++)
    {    
        iArea=iArea+(vecPoly[iCycle].x*vecPoly[(iCycle+1) % iCount].y-vecPoly[(iCycle+1) % iCount].x*vecPoly[iCycle].y);
    }
    
    return abs(0.5*iArea);
}

void detect(Mat &frame, flag_detection_t *td)
{
	double detect_begin1 = clock();
    //step 1. Binarize the frame with adaptive threshold
    Mat binarition;
    Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    IplImage frame2 = IplImage(frame);
    int adaptThresh = otsuThreshold(&frame2);
	cout << "zyx Time is " << ( clock() - detect_begin1 ) / CLOCKS_PER_SEC * 1000 << endl;
    threshold(gray, binarition, adaptThresh, 255, CV_THRESH_BINARY);

	double detect_begin2 = clock();
    //step 2. find the squares in the binarization picture
	CvMemStorage* storage = cvCreateMemStorage(0);     // the storage to save contours
	CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );
	vector<Point> squares_centers;           //the center points of the squares
	vector< vector<Point> > squares_v;       //the corner points of the squares
	//vector< vector<Point> > squares_useful;  //the corner points of the useful squares
	IplImage binarition2 = IplImage(binarition);
	FindSquares( &binarition2, squares, storage, squares_centers, squares_v, cvPoint(0, 0));

	double detect_begin3 = clock();
    //step 3. classify the squares according to the center of squares.
    //If some centers are close enough, we think they belong to one group.
	vector< vector<int> > squares_class_index;
	CenterClassification(squares_centers, squares_class_index);

	double detect_begin4 = clock();
    //step 4. find out the largest group.
    //It is really possible that the largest group is our aim.
    //If the number of the group' members is larger than MIN_SQUARE_NUM, we think we get our aim.
	int squares_centers_cnum = 0;
	int squares_centers_cindex;
	Point flag_center = cvPoint(0, 0);
	for(int i = 0; i < squares_class_index.size(); i++)
	{
		if( squares_class_index[i].size() > squares_centers_cnum )
		{
			squares_centers_cnum = squares_class_index[i].size();
			squares_centers_cindex = i;
		}
	}
    //if the number of class members is larger than MIN_SQUARE_NUM   
    if( squares_centers_cnum >= MIN_SQUARE_NUM )     
		{
            td->result = 1;
            //compute average of the center of squares which is our aim point
			for(int i = 0; i < squares_class_index[squares_centers_cindex].size(); i++)
		    {
			flag_center.x = flag_center.x + squares_centers[squares_class_index[squares_centers_cindex][i]].x;
			flag_center.y = flag_center.y + squares_centers[squares_class_index[squares_centers_cindex][i]].y;
		    }
            td->center.x = flag_center.x / squares_class_index[squares_centers_cindex].size();
            td->center.y = flag_center.y / squares_class_index[squares_centers_cindex].size();

		    for(int i = 0; i < squares_class_index[squares_centers_cindex].size(); i++)
		    {
			//squares_useful.push_back(squares_v[squares_class_index[squares_centers_cindex][i]]);
            td->corners.push_back(squares_v[squares_class_index[squares_centers_cindex][i]]);
		    }

            //step 5. the squares' corners are unordered, so we should order them.
	        //we make them clockwise.
	        for( int i = 0; i < td->corners.size(); i++)
			    RankPoint(td->corners[i]);

            //step 6. Use the function solvePnP to compute the position of camera
			/*Mat Rod_r, PP_w;
	        vector<Point2f> corners2d;
	        Point temp2d;
	        for(int i = 1; i < 3; i++)
		    for(int j = 0; j < 4; j++)
		    {
			    temp2d = td->corners[td->corners.size()-i][j];
			    corners2d.push_back(temp2d);
		    }
	        solvePnP(td->points3d, corners2d, K, td->distort, Rod_r, td->t, false);
	        Rodrigues(Rod_r, td->R);
	        PP_w = -td->R.inv() * td->t;
			//cout << "Camera center:" << endl << PP_w.t() << endl;
			f7 << PP_w.at<double>(0,0) << endl;
			f8 << PP_w.at<double>(1,0) << endl;
			f9 << PP_w.at<double>(2,0) << endl;
			td->P_w.x = PP_w.at<double>(0,0);
			td->P_w.y = PP_w.at<double>(1,0);
			td->P_w.z = PP_w.at<double>(2,0);*/
			//cout << "t:" << endl << td->t.t() << endl;
		}
    else
        td->result = 0;
	
	cout << "Step1 Time is " << ( detect_begin2 - detect_begin1 ) / CLOCKS_PER_SEC * 1000 << endl;
	cout << "Step2 Time is " << ( detect_begin3 - detect_begin2 ) / CLOCKS_PER_SEC * 1000 << endl;
	cout << "Step3 Time is " << ( detect_begin4 - detect_begin3 ) / CLOCKS_PER_SEC * 1000 << endl;
	cout << "Step4 Time is " << ( clock() - detect_begin4 ) / CLOCKS_PER_SEC * 1000 << endl;
	cvReleaseMemStorage(&storage);
}

void EstimatePose( flag_detection_t *td, Euler_t current_att)
{
	double detect_begin = clock();
	double gama = acos(cos(current_att.pitch) * cos(current_att.roll));
	float delta_l, L, H, X, Y;
	bool direction;

	if(gama / PI * 180 < 1)
		gama = 0;

	td->img_area = ComputeArea(td->corners[td->corners.size()-1]);
	td->Pc.z = sqrt(td->square_area * K.at<float>(0,0) * K.at<float>(1,1) / (td->img_area * cos(current_att.pitch) * cos(current_att.roll)));
	td->Pc.x = (td->center.x - K.at<float>(0,2)) * td->Pc.z / K.at<float>(0,0);
	td->Pc.y = (td->center.y - K.at<float>(1,2)) * td->Pc.z / K.at<float>(1,1);
	Oc.x = K.at<float>(0,2) + K.at<float>(0,0) * tan(current_att.roll);
	Oc.y = K.at<float>(1,2) - K.at<float>(1,1) * tan(current_att.pitch);
	delta_l = sqrt(td->Pc.x*td->Pc.x + td->Pc.y*td->Pc.y) * tan(fabs(gama));
	if( ((Oc.x-K.at<float>(0,2)) * (td->center.x-K.at<float>(0,2)) + (Oc.y-K.at<float>(1,2)) * (td->center.y-K.at<float>(1,2))) > 0 )
	{
		L = td->Pc.z - delta_l;
		direction = 1;
	}
	else
	{
		L = td->Pc.z + delta_l;
		direction = 0;
	}
	H = L * cos(gama);
	if(direction == 0)
	{
		Y = H * tan( current_att.pitch + atan((td->center.y - K.at<float>(1,2)) /  K.at<float>(1,1)) );
		X = H * tan( current_att.roll - atan((td->center.x - K.at<float>(0,2)) /  K.at<float>(0,0)) );
	}
	else
	{
		Y = H * tan( current_att.pitch + atan((td->center.y - K.at<float>(1,2)) /  K.at<float>(1,1)) );
		X = H * tan( current_att.roll - atan((td->center.x - K.at<float>(0,2)) /  K.at<float>(0,0)) );
	}
	//cout << X << "  " << Y << "  " << H << endl;
	f1 << X << endl;
	f2 << Y << endl;
	f3 << H << endl;
	//f7 << current_att.pitch / 3.1415926 * 180 << endl;
	//f8 << current_att.roll / 3.1415926 * 180 << endl;
	//f9 << current_att.yaw / 3.1415926 * 180 << endl;
	td->Pw.x = X;
	td->Pw.y = Y;
	td->Pw.z = H;
	//cout << td->Pc.x << "  " << td->Pc.y << "  " << td->Pc.z << endl;
	//cout << current_att.pitch / 3.1415926 * 180 << "  " << current_att.roll / 3.1415926 * 180 << "  " << current_att.yaw / 3.1415926 * 180 << endl;
	//cout << H << endl;
	double duration = ( clock() - detect_begin ) / CLOCKS_PER_SEC * 1000;
	cout << "Estimate Time is " << duration << endl;
}

flag_detection_t *flag_detection_create()
{
    flag_detection_t *td = (flag_detection_t*) calloc(1, sizeof(flag_detection_t));

    //save the distortion yaml of camera
    td->distort.push_back(0.0402);
	td->distort.push_back(-0.126);
	td->distort.push_back(0);
	td->distort.push_back(0);
	/*td->distort.push_back(0);
	td->distort.push_back(0);
	td->distort.push_back(0);
	td->distort.push_back(0);*/

    //save the world position of the corners
    //Read3dPoints(td->points3d);

	td->img_center.x = 376;
	td->img_center.y = 240;

	td->square_area = 10000;

    return td;
}

void flag_detection_destroy(flag_detection_t *td)
{
    free(td);
}

void flag_detection_clear(flag_detection_t *td)
{
    td->corners.clear();
}
