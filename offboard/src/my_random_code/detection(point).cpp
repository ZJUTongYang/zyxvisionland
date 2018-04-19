#include "detection.h"

#define PI 3.1415926535879
#define IMG_AREA 1228800
#define MIN_SQUARE_AREA 30
#define MIN_CENTER_DIS 10
#define MIN_SQUARE_NUM 3

cv::Mat K( cv::Matx33f(315.5, 0, 376.5,
						 0, 315.5, 240.5,
						 0, 0, 1) );

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
	std::ifstream reader("/home/zhengyuxin/zhazha/control_ws/src/offboard/axis.txt");

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

void detect(Mat &frame, flag_detection_t *td)
{
    int width = frame.rows;
	int height = frame.cols;
	int img_area = width * height;

    //step 1. Binarize the frame with adaptive threshold
    Mat binarition;
    Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    IplImage frame2 = IplImage(frame);
    int adaptThresh = otsuThreshold(&frame2);
    threshold(gray, binarition, adaptThresh, 255, CV_THRESH_BINARY);

    //step 2. find the squares in the binarization picture
	CvMemStorage* storage = cvCreateMemStorage(0);     // the storage to save contours
	CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );
	vector<Point> squares_centers;           //the center points of the squares
	vector< vector<Point> > squares_v;       //the corner points of the squares
	//vector< vector<Point> > squares_useful;  //the corner points of the useful squares
	IplImage binarition2 = IplImage(binarition);
	FindSquares( &binarition2, squares, storage, squares_centers, squares_v, cvPoint(0, 0));

    //step 3. classify the squares according to the center of squares.
    //If some centers are close enough, we think they belong to one group.
	vector< vector<int> > squares_class_index;
	CenterClassification(squares_centers, squares_class_index);

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
	        Mat Rod_r;
	        vector<Point2f> corners2d;
	        Point temp2d;
	        for(int i = 1; i < 4; i++)
		    for(int j = 0; j < 4; j++)
		    {
			    temp2d = td->corners[td->corners.size()-i][j];
			    corners2d.push_back(temp2d);
		    }
	        solvePnP(td->points3d, corners2d, K, td->distort, Rod_r, td->t, false);
	        Rodrigues(Rod_r, td->R);
	        /*cout << "r:" << endl << Rod_r << endl;
	        cout << "R:" << endl << R << endl;
	        cout << "t:" << endl << t << endl;*/
	        td->P_w = -td->R.inv() * td->t;
	        cout << "Camera center:" << endl << td->P_w.t() << endl;
		}
    else
        td->result = 0;
}

flag_detection_t *flag_detection_create()
{
    flag_detection_t *td = (flag_detection_t*) calloc(1, sizeof(flag_detection_t));

    //save the distortion yaml of camera
    /*td->distort.push_back(-0.1018);
	td->distort.push_back(0.4201);
	td->distort.push_back(0);
	td->distort.push_back(0);*/
	td->distort.push_back(0);
	td->distort.push_back(0);
	td->distort.push_back(0);
	td->distort.push_back(0);

    //save the world position of the corners
    Read3dPoints(td->points3d);

	td->img_center.x = 376;
	td->img_center.y = 240;
	
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
