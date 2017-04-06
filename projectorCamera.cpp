#include "projectorCamera.h"
#define SHOW_INFO
#include <iostream>
#include <fstream>
using namespace std;


/*!
@function              sort points
@abstract              point comparison by location
@discussion
@param
@result
*/
bool cmpPoint(Point2f& pt1, Point2f& pt2)
{
	//high priority sort y
	if (abs(pt1.y - pt2.y) > 10)
	{
		return pt1.y <= pt2.y;
	}

	//default priority sort x
	return pt1.x <= pt2.x;
}

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
ProjectorCamera::ProjectorCamera()
{	
	//LOGF("Processing flow : %s", "ProjectorCamera constructor start");
	calibrated = false;
	initBg = false;
	initFrames = 50;
	frameID = 0;
	nearSurFaceThresh = 4;
	nearCameraThresh = 1000;
	histMinPixs = 15;

	//calibration
	gridstart_x = 25;
	gridstart_y = 47;
	gridwidth = 50;
	offset_x = 0;
	offset_y = 0;

	//
	screenRoi   = Rect(100, 100, 50, 300);
	filterRoi   = Rect(48, 35, 40, 30);

	//zhangbo
	ncount_totalimg = 0;
    ncount_grasp = 0;

    ncount_totalimg_open = 0;
    ncount_open = 0;

    m_graspstate = false;
    m_openstate = false;
    m_wavestate = 0;

    //calibration result data
    FileStorage fsin;
    //LOGD("try to open cascade_zfz.xml, %s" , cablibParaFilePath.c_str());
    fsin.open("cascade_zfz.xml", FileStorage::READ);
    if (!fsin.isOpened())
    {
       //LOGD("failed to open file cascade_zfz.xml");
    	return ;
    }
    else
    {
        fsin["R"] >> camToProR;
        fsin["T"] >> camToProT;
        fsin["camera_KK"] >> camera_KK;
        fsin["camera_dis"] >> camera_dis;
        fsin["project_KK"] >> project_KK;
        fsin["project_dis"] >> project_dis;
        //LOGD("success to open file cascade_zfz.xml");
    }
    fsin.release();
}

/*
*function: default destructor
*note    :
*/
ProjectorCamera::~ProjectorCamera()
{
    //LOGF("Processing flow : %s", "ProjectorCamera destructor start");
}

/*
*function:judge object depth distribution is continue or not
*note    :
*/
bool ProjectorCamera::histcontinue(Mat& object)
{
    //LOGF("Processing flow : %s", "histcontinue start");

	//init flag parameters
	bool iscontinue(false);
	size_t front(0), behind(20);

	//hist parameters setting
	int channel[1] = { 0 };
	const int histSize[1] = { 4 };
	float hranges[2] = { 5, 13 };
	const float* ranges[1] = { hranges };
	calcHist(&object, 1, channel, Mat(), hist, 1, histSize, ranges);

	cout << hist.t() << endl;
	//checkout hist is continue or not
	for (int row = 1; row < hist.rows; row++)
	{
		if (hist.at<float>(row, 0) >= 4)
		{
			front = row;
		}
		else
		{
			break;
		}
	}

	return front>=hist.rows-1;
}

/*
*function: initialization
*note    :
*/
void ProjectorCamera::init()
{

}


/*
*function: int->string
*note    :
*/
string ProjectorCamera::intTostring(int num)
{
	stringstream ss;
	ss << num;
	string out;
	ss >> out;

	return out;
}

/*
*function: using fixed calibration parameters
*note
*/
void ProjectorCamera::calibrationFixed()
{
    //LOGF("Processing flow : %s", "calibrationFixed start");

    h = Mat(3,3,CV_64F,Scalar(0));
    h = (Mat_<double>(3,3) << 2.5797362305397424e+000, -3.7274467612214901e-001,
                                 4.2250897290272846e+001, 5.4430909225976175e-003,
                                 2.5050769943386775e+000, -4.7408720715676232e+001,
                                 2.5509261309249496e-005, -4.8136826315280307e-004, 1.);
    h = h.inv();
    calibrated = true;
    //fd = EVENT::opendev();
    return;
}

/*
*function: calibrate the homography between projector and camera
*note    :
*/
void ProjectorCamera::calibrationPerspective()
{
    //LOGF("Processing flow : %s", "calibrationPerspective start");
    //if (colorImg.empty())
    //{
    //	//LOGW("Processing warn : %s", "calibration empty colorImg");
    //	return;
    //}

    //image setting
    Rect Roi(20, 110, 500, 320);
    h = Mat::eye(3, 3, CV_64F);
    h.at<double>(0,0) = -2.52428 ;
    h.at<double>(0,1) = -0.390762;
    h.at<double>(0,2) = 1308.99;
    h.at<double>(1,0) = -0.00548842;
    h.at<double>(1,1) = -3.06378;
    h.at<double>(1,2) = 752.149;
    h.at<double>(2,0) = -5.30545e-006;
    h.at<double>(2,1) = -0.000737592;
    h.at<double>(2,2) = 1;
    screenRoi = Roi;
    h = (Mat_<double>(3,3) << 2.5797362305397424e+000, -3.7274467612214901e-001,
                                     4.2250897290272846e+001, 5.4430909225976175e-003,
                                     2.5050769943386775e+000, -4.7408720715676232e+001,
                                     2.5509261309249496e-005, -4.8136826315280307e-004, 1.);
    //save corner points image
    //imwrite("/sdcard/calibration.jpg", colorImg(Roi));
    calibrated = true;
    //fd = EVENT::opendev();

    //LOGC("calibration stage : %s", "success");
}

/*
*function: calibrate the homography between projector and camera
*note    :
*/
void ProjectorCamera::calibration()
{
    //LOGF("Processing flow : %s", "calibration start");
	//check input image	
	if (colorImg.empty())
	{
		//LOGW("Processing warn : %s", "calibration empty colorImg");
		return;
	}

    //image setting
    colorImg = colorImg(screenRoi);
	Mat src  = colorImg,src_gray;
	cvtColor(src, src_gray, 6);

	//Corner finding parameters
	double qualityLevel = 0.01;
	double minDistance = 14;
	int    blockSize = 3;
	double k = 0.04;
	bool useHarrisDetector = false;
	vector<Point2f> corners;

	//Apply corner detection :Determines strong corners on an image.
	goodFeaturesToTrack(src_gray,
		corners,
		400,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);

	if(corners.size()<100) return;

	//Draw corners detected
	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].x < filterRoi.x - corners[i].y / 35 ||
		    corners[i].y < filterRoi.y ||
            corners[i].x > src.cols - filterRoi.width + corners[i].y  / 35||
            corners[i].y > src.rows - filterRoi.height)
        {
			corners.erase(corners.begin() + i);
			i--;
			continue;
		}
		circle(src, corners[i], 1, Scalar(0, 0, 255));
	}//End all points

	//save corner points image
    imwrite("/sdcard/calibration.jpg", src);

	//sort corner points
	sort(corners.begin(), corners.end(), cmpPoint);

	//calculate the screen corresponding points
	vector<Point2f> vscreenPoints;
	vscreenPoints.push_back(Point2f(gridstart_x, gridstart_y));
	Point2f lastRowStartPoint(corners[0]);
    for (int i = 1; i < corners.size(); i++)
    {
    	Point2f last = vscreenPoints[i - 1];
    	if (corners[i].y - corners[i-1].y > 10)
    	{
    		int scalar = 0;
    		if (corners[i].x - lastRowStartPoint.x < -10) continue;
    		else if (corners[i].x - lastRowStartPoint.x < 10)
    		{
    			scalar = 0;
    			lastRowStartPoint.x = (lastRowStartPoint.x + corners[i].x) / 2;
    		}
    		else if (corners[i].x - lastRowStartPoint.x < 30)
    		{
    			scalar = 1;
    		}
    		else return;
    		last.x = gridstart_x + (gridwidth * scalar);
    		last.y += gridwidth;
    		vscreenPoints.push_back(last);
    	}
    	else
    	{
    		int scalar = 1;
    		if (corners[i].x - corners[i - 1].x > 30) scalar = 2;
    		if (corners[i].x - corners[i - 1].x > 50) scalar = 3;
    		last.x += gridwidth * scalar;
    		vscreenPoints.push_back(last);
    	}
    }

	//calculate homography
	if(corners.size()>100)
	{
	     h = cv::findHomography(corners, vscreenPoints, LMEDS);//LMEDS RANSAC
	     //LOGC("calibration stage homography1: %f", h.at<double>(0,0));
	     //LOGC("calibration stage homography2: %f", h.at<double>(0,1));
	     /*LOGC("calibration stage homography3: %f", h.at<double>(0,2));
	     LOGC("calibration stage homography1: %f", h.at<double>(1,0));
         LOGC("calibration stage homography2: %f", h.at<double>(1,1));
         LOGC("calibration stage homography3: %f", h.at<double>(1,2));
         LOGC("calibration stage homography1: %f", h.at<double>(2,0));
         LOGC("calibration stage homography2: %f", h.at<double>(2,1));
         LOGC("calibration stage homography3: %f", h.at<double>(2,2));*/
	}
	else
	{
	     return;
	}

	//test homegraphy
	vector<Point2f> testH(corners.size());
	perspectiveTransform(corners, testH, h);
	float maxsub(0);
	for (int i = 0; i < corners.size(); i++)
	{
		float sub = max(abs(vscreenPoints[i].x - testH[i].x),
		                abs(vscreenPoints[i].y - testH[i].y));
		if(maxsub < sub) maxsub = sub;
		if(maxsub > 6)
		{
			/*LOGC("calibration stage : %s", " failed");
			LOGC("calibration stage max distance: %f", maxsub);*/
			return;
		}
	}

	calibrated = true;
	//fd = EVENT::opendev();

	//LOGC("calibration stage : %s", "success");
    //LOGC("calibration stage max distance: %f", maxsub);
}

/*
*function: get background depth
*note
*/
void ProjectorCamera::getBgDepth()
{
	//average image
	if (frameID <= initFrames)
	{
		//accumulate depth image
		bgdepths.push_back(depthImg);
	}
	else if (frameID == initFrames + 1)
	{
	    averaImg = Mat(depthImg.rows, depthImg.cols,CV_32F);
        //every pixel
        for (int row = 0; row < depthImg.rows; row++)
		{
			for (int col = 0; col < depthImg.cols; col++)
			{
			    //get valid depth data
				int  validnum = 0;
				float sum = 0;
				for (auto bg : bgdepths)
				{
					float depth = bg.at<float>(row, col);
					if (depth > 600)
					{
						validnum++;
						sum += depth;
					}
				}

                //get average depth
				if (validnum > 0)
				{
					averaImg.at<float>(row, col) = sum / validnum;
				}
				else
				{
					averaImg.at<float>(row, col) = 0;
				}
			}
		}
		//averaImg /= initFrames;
		initBg = true;
		nearCameraThresh = averaImg.at<float>(averaImg.rows / 2,averaImg.cols /2) - 250;

		//LOGF("Processing flow nearCameraThresh: %f", nearCameraThresh);

		//LOGF("Processing flow : %s", "getBgDepth success");

		//LOGD("nativeStart caught initTime: %f", getBg.getClock());
	}
}

/*
*function:get foreground object
*note    :
*/
void ProjectorCamera::getFgDepth()
{
    //LOGF("Processing flow : %s", "getFgDepth start");

	//get absolute depth image
	foreground = abs(depthImg - averaImg);
	foreground.copyTo(diffeImg);
	threshold(foreground, foreground, nearSurFaceThresh, 255, CV_THRESH_TOZERO);
    //cvtColor(foreground,foreground_copy,8);
    //imwrite("/sdcard/foreground_copy.jpg",foreground_copy);
	//LOGD("nativeStart caught : %s", "getFgDepth success");
}
/*
*function: main processing flow
*note
*/
void ProjectorCamera::processing(Mat& colorSrc, Mat& depthSrc)
{
    //LOGF("Processing flow : %s", "processing start");
	CVTIME processingTotalTime,initTime;
	
	//check input image
	if (depthSrc.empty())
	{
		//LOGW("Processing warn : %s", "empty depthImg");
		return;
	}

	if (depthSrc.rows != 480 || depthSrc.cols != 640)
    {
    	//LOGW("Processing warn : %s", "error size depthImg");
    	return;
    }

	//depth image preprocessing
	frameID++;
	screenRoi   = Rect(80, 80, 400, 250);
	depthImg = depthSrc(screenRoi);
	colorImg = colorSrc(screenRoi);
	depthImg.convertTo(depthImg, CV_32F);

	if (!initBg)
	{
	    getBgDepth();
	    return;
	}
	//LOGD("nativeStart caught getBgDepthTime: %f", getBg.getClock());
	//get foreground depth
	CVTIME getFgDepthTime;
	getFgDepth();
	//LOGD("nativeStart caught getFgDepthTime: %f", getFgDepthTime.getClock());

    //get foreground
    CVTIME getContourTime;
    getContour();
    //LOGD("nativeStart caught getContourTime: %f", getContourTime.getClock());

	CVTIME findFingerTime;
	findFinger();
	//LOGD("nativeStart caught findFingerTime: %f", findFingerTime.getClock());

    //LOGD("appState: %d", appState);
	appState = 1;
    if(appState == 1)
    {
        CVTIME findOnDeskObjectTime;
	    findOnDeskObject();
	    //findInAirObject();
	    //LOGD("nativeStart caught cake findOnDeskObjectTime: %f", findOnDeskObjectTime.getClock());
	}
	else if(appState == 2)
	{
	    CVTIME findOnDeskObjectTime;
	    findOnDeskObject();
	    findInAirObject();
	    //LOGD("nativeStart caught water findOnDeskObjectTime: %f", findOnDeskObjectTime.getClock());
	}
	else if(appState == 3)
	{
        CVTIME findInAirGestureTime;
        findInAirGesture();
        //LOGD("nativeStart caught findInAirGestureTime: %f", findInAirGestureTime.getClock());
	}
    depthImg.release();
    depthSrc.release();
	//LOGF("Processing flow processingTotalTime: %f", processingTotalTime.getClock());
}

/*
*function: find contour information in foreground image
*note    :
*/
void ProjectorCamera::getContour()
{
    //detect contours in foreground image
	Mat fgImage,binaryIm;
	foreground.copyTo(fgImage);
	convetFloatToChar(fgImage,binaryIm);

	std::vector< std::vector<Point> > contours;
	findContours(binaryIm, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	bincp = binaryIm;

    objects.clear();
    for (int i = 0; i < contours.size(); i++)
    {
        //get contour area
		vector<Point> contour = contours[i];
		Mat contourMat = Mat(contour);
		double cArea = contourArea(contourMat);

        //min object area
        if (cArea < 300) continue;

        //fill hole
		Rect rec = boundingRect(contours[i]);
		Mat object = foreground(rec);

        //exclude static object
		double mindepth;
		minMaxLoc(object, &mindepth, &maxdepth);

        ObjectInfo objectInfo;
        objectInfo.object   = object;
        objectInfo.maxdepth = maxdepth;
        objectInfo.contour  = contour;
        objectInfo.cArea    = cArea;

        objects.push_back(objectInfo);
    }
}

/*
*function: detection projection object from foreground image
*note    :
*/
void ProjectorCamera::findInAirObject()
{
    //LOGF("Processing flow : %s", "findInAirObject start");
    for (int ind = 0; ind < objects.size(); ind++)
    {
        if(objects[ind].cArea<12000) continue;
    	if (objects[ind].maxdepth < 100 ) continue;

    	//LOGD("stereo projection cArea: %f, %d", objects[ind].cArea, frameID);
        //LOGD("stereo projection maxdepth: %f", objects[ind].maxdepth);

        //将轮廓转化为多边形
        HandContour temHand;
        Mat contourMat = Mat(objects[ind].contour);
        approxPolyDP(contourMat, temHand.approxCurve, 12, true);

        //剔除四个深度图中的顶点
        for (int ind = 0; ind < temHand.approxCurve.size(); ind++)
		{
			if ((temHand.approxCurve[ind].x % depthImg.cols) < 10 &&
				(temHand.approxCurve[ind].y % depthImg.rows) < 10)
			{
				temHand.approxCurve.erase(temHand.approxCurve.begin() + ind);
				ind--;
			}
		}
        //LOGD("stereo projection approxCurve.size : %d", temHand.approxCurve.size());
        if(temHand.approxCurve.size() < 6) continue;
        Point armCenter;
		//寻找手臂部分
		for (int ind = 0; ind < temHand.approxCurve.size();ind++)
		{
			vector<Point> & curve = temHand.approxCurve;
			int pointNum(curve.size()),
				LfristId((ind + 1)  % pointNum),
				LsecondId((ind + 2) % pointNum),
				LthirdId((ind + 3)  % pointNum);

			int nearThresh = 10;
			if (((depthImg.cols - curve[LfristId].x) < nearThresh || curve[LfristId].x < nearThresh ||
				(depthImg.rows - curve[LfristId].y) < nearThresh || curve[LfristId].y < nearThresh) &&
				((depthImg.rows - curve[LsecondId].y) < nearThresh || curve[LsecondId].y < nearThresh ||
				(depthImg.cols - curve[LsecondId].x) < nearThresh || curve[LsecondId].x < nearThresh) )
		    {
		        armCenter.x = (curve[ind] + curve[LthirdId]).x / 2.0;
                armCenter.y = (curve[ind] + curve[LthirdId]).y / 2.0;
                temHand.approxCurve.erase(temHand.approxCurve.begin() + LfristId);
				if (LsecondId > LfristId) LsecondId--;
				temHand.approxCurve.erase(temHand.approxCurve.begin() + LsecondId);
				break;
		    }
		}

        //select valid points
		while (temHand.approxCurve.size() > 4)
		{
			float mindis(10000);
			int   index(0);
			for (int ind = 0; ind < temHand.approxCurve.size(); ind++)
			{
				float temdis = calPointDistance(armCenter, temHand.approxCurve[ind]);
				if (temdis < mindis)
				{
					index = ind;
					mindis = temdis;
				}
			}
			temHand.approxCurve.erase(temHand.approxCurve.begin() + index);
		}

        if (temHand.approxCurve.size() == 4)
		{
		    //LOGD("stereo projection point dis: %f %f", abs(calPointDistance(temHand.approxCurve[0], temHand.approxCurve[1])
            //- calPointDistance(temHand.approxCurve[2], temHand.approxCurve[3])), abs(calPointDistance(temHand.approxCurve[2], temHand.approxCurve[1])
            //- calPointDistance(temHand.approxCurve[0], temHand.approxCurve[3])));

            Scalar center = mean(Mat(temHand.approxCurve));
            Point centerPoint = Point(center.val[0], center.val[1]);
            if(foreground.at<float>(centerPoint.y,centerPoint.x) < 60) continue;
		    //is a rectangle
			if (abs(calPointDistance(temHand.approxCurve[0], temHand.approxCurve[1])
				- calPointDistance(temHand.approxCurve[2], temHand.approxCurve[3])) > 50 ||
				abs(calPointDistance(temHand.approxCurve[2], temHand.approxCurve[1])
				- calPointDistance(temHand.approxCurve[0], temHand.approxCurve[3])) > 50)
				continue;
		}
		else
		{
		    continue;
		}

        clockwiseContour(temHand.approxCurve);
        vector<Point2f> lastVertex = stereoProjectHover.cameraVertex;
        vector<float>   lastDepth  = stereoProjectHover.vertexDepth;
        stereoProjectHover.cameraVertex.clear();
		stereoProjectHover.vertexDepth.clear();
		vector<Point2f> vpoints;
		for (auto point : temHand.approxCurve)
		{
			vector<float> vdepth;
			int search_w(20),dis(1000);
			float validDepth(0);
			float bgdepth = averaImg.at<float>(point.y,point.x) - 50;
			CVTIME findPointsTime;
			for (int row = max(0, point.y - search_w); row < min(depthImg.rows - 1, point.y + search_w); row++)
			{
				for (int col = max(0, point.x - search_w); col<min(depthImg.cols - 1, point.x + search_w); col++)
				{
					float depht = depthImg.at<float>(row, col);
					int temdis = abs(point.y -row) + abs(point.x - col);
					if (depht>200 && depht<bgdepth && temdis<dis)
					{
					    dis = temdis;
					    validDepth = depht;
					}
				}
			}
			//LOGD("stereo projection findPointsTime: %f", findPointsTime.getClock());

			if (validDepth > 0)
			{
                float Z = validDepth;
				float y = point.y;
                float x = point.x;

                for(int subind = 0;subind<lastDepth.size();subind++)
                {
                    float dis = calPointDistance(lastVertex[subind],Point2f(x,y));
                    if( dis< 5 && dis>10)
                    {
                        x = (x + lastVertex[subind].x) / 2;
                        y = (y + lastVertex[subind].y) / 2;
                        Z = (Z + lastDepth[subind]) / 2;
                        break;
                    }
                    else if(dis<5)
                    {
                        x = lastVertex[subind].x;
                        y = lastVertex[subind].y;
                        Z = lastDepth[subind];
                        break;
                    }
                }
				stereoProjectHover.cameraVertex.push_back(Point2f(x,y));
				stereoProjectHover.vertexDepth.push_back(Z);
			}
			else
			{
			    //LOGD("stereo projection error: %s", "no valid depth pixel");
			}
		}

			cout<<"stereoProjectHover.proVertex3D.size: "<<stereoProjectHover.cameraVertex.size()<<endl;
			for(int kkk=0; kkk<stereoProjectHover.cameraVertex.size(); ++kkk)
				circle(colorImg, Point(stereoProjectHover.cameraVertex[kkk].x,stereoProjectHover.cameraVertex[kkk].y), 3, Scalar(0,0,255), -1);
			imshow("colorImg",colorImg);
	        waitKey(1);
			return;
        //坐标转换
        if(stereoProjectHover.cameraVertex.size() > 2)
        {
            vector<Point3f> lastVertex3D = stereoProjectHover.proVertex3D;
            stereoProjectHover.valid = true;
            stereoProjectHover.proVertex3D.clear();

            pointCamToPro(stereoProjectHover.cameraVertex,stereoProjectHover.vertexDepth,stereoProjectHover.proVertex3D);
            stereoProjectHover.center = stereoProjectHover.cameraVertex[0];
            //refineVerticals(stereoProjectHover.proVertex3D);

            for(int ind=0;ind<stereoProjectHover.proVertex3D.size();ind++)
            {
                Point3f& point = stereoProjectHover.proVertex3D[ind];
                for(int subind=0;subind<lastVertex3D.size();subind++)
                {
                    float dis = calPointDistance(lastVertex3D[subind],point);
                    if(dis> 10 && dis<15)
                    {
                        //LOGD("stereo projection dis: dis%f", dis);
                        point.x = (lastVertex3D[subind].x + point.x) / 2;
                        point.y = (lastVertex3D[subind].y + point.y) / 2;
                        point.z = (lastVertex3D[subind].z + point.z) / 2;
                        break;
                    }
                    else if(dis <=10)
                    {
                        //LOGD("stereo projection dis: dis%f", dis);
                        point = lastVertex3D[subind];
                        break;
                    }
                }
            }

			
            //LOGD("stereo projection in the air point: %d x%f y%f z%f x%f y%f z%f x%f y%f z%f x%f y%f z%f dis%f dis%f",frameID,
            //stereoProjectHover.proVertex3D[0].x,stereoProjectHover.proVertex3D[0].y,stereoProjectHover.proVertex3D[0].z,
            //stereoProjectHover.proVertex3D[1].x,stereoProjectHover.proVertex3D[1].y,stereoProjectHover.proVertex3D[1].z,
            //stereoProjectHover.proVertex3D[2].x,stereoProjectHover.proVertex3D[2].y,stereoProjectHover.proVertex3D[2].z,
            //stereoProjectHover.proVertex3D[3].x,stereoProjectHover.proVertex3D[3].y,stereoProjectHover.proVertex3D[3].z,
            //calPointDistance(stereoProjectHover.proVertex3D[0],stereoProjectHover.proVertex3D[2]),
            //calPointDistance(stereoProjectHover.proVertex3D[1],stereoProjectHover.proVertex3D[3])
            //);
            //imwrite("/sdcard/depthImg.jpg",depthImg-700);
            return;
        }
    }
}


/*
*function: detection projection object from foreground image
*note    :
*/
void ProjectorCamera::findOnDeskObject()
{
	vector<Point2f> propoint(4);
    //LOGF("Processing flow : %s", "findOnDeskObject start");
    for (int ind = 0; ind < objects.size(); ind++)
    {
    	if(objects[ind].cArea<6500 || objects[ind].cArea>20000) continue;
    	//LOGD("stereo projection cArea: %f, %d", objects[ind].cArea, frameID);
    	//LOGD("stereo projection maxdepth: %f", objects[ind].maxdepth);
    	//if (objects[ind].maxdepth < 60 ) continue;

        //将轮廓转化为多边形
        HandContour temHand;
        Mat contourMat = Mat(objects[ind].contour);
        approxPolyDP(contourMat, temHand.approxCurve, 15, true);
        if(temHand.approxCurve.size() != 4) continue;
        //LOGD("stereo projection vertex size: %d", temHand.approxCurve.size());
		//clock_t end = clock();
		Point2f centerPoint;
        if (temHand.approxCurve.size() == 4)
		{
		    //LOGD("stereo projection point dis: %f %f", abs(calPointDistance(temHand.approxCurve[0], temHand.approxCurve[1])
      //      - calPointDistance(temHand.approxCurve[2], temHand.approxCurve[3])), abs(calPointDistance(temHand.approxCurve[2], temHand.approxCurve[1])
      //      - calPointDistance(temHand.approxCurve[0], temHand.approxCurve[3])));

            Scalar center = mean(Mat(temHand.approxCurve));
            centerPoint = Point(center.val[0], center.val[1]);
            if(foreground.at<float>(centerPoint.y,centerPoint.x) > 100) continue;
		    //is a rectangle
			if (abs(calPointDistance(temHand.approxCurve[0], temHand.approxCurve[1])
				- calPointDistance(temHand.approxCurve[2], temHand.approxCurve[3])) > 50 ||
				abs(calPointDistance(temHand.approxCurve[2], temHand.approxCurve[1])
				- calPointDistance(temHand.approxCurve[0], temHand.approxCurve[3])) > 50)
				continue;
		}

        //获取四个顶点
        RotatedRect rorec = minAreaRect(objects[ind].contour);
        Point2f ppt[4];
        rorec.points(ppt);
        vector<Point2f> vpoints(ppt,ppt+4);

		vpoints[0] = temHand.approxCurve[0];
		vpoints[1] = temHand.approxCurve[1];
		vpoints[2] = temHand.approxCurve[2];
		vpoints[3] = temHand.approxCurve[3];
        clockwiseContour(vpoints);
		vpoints.push_back(centerPoint);

		for (int i = 0; i < vpoints.size(); ++i)
			cout << vpoints[i].x <<" "<< vpoints[i].y <<" "<< depthImg.at<float>(vpoints[i].y, vpoints[i].x) << endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>("five.pcd", *target) == -1)//打开点云文件
		{
				PCL_ERROR("Couldn't read file target.pcd\n");
				return;
		}

		double fx = 506.13;
		double fy = 505.915;
		double cx = 306.48;
		double cy = 237.17;
		Eigen::Matrix<double, 3, 3> DepToColR;
		Eigen::Matrix<double, 3, 1> DepToColT;

		DepToColR << 9.9999990604439426e-001, 4.0370037164641460e-004, -1.5791520725434802e-004,
			-4.0351723215206639e-004, 9.9999924804573248e-001, 1.1580508680105257e-003,
			1.5838259407514551e-004, -1.1579870376978076e-003, 9.9999931699025391e-001;

		DepToColT << -2.5602946386750983e+001, -2.0611612572279053e-001, 2.9436110127104387e+000;
		Eigen::Matrix<double, 3, 1> Tmp;
		Eigen::Matrix<double, 3, 1> PointInCol;
		source->width = 5;
		source->height = 1;
		source->is_dense = false;
		source->points.resize(source->width * source->height);
		for (size_t i = 0; i < source->points.size(); ++i)
		{		
			double depthVal = depthImg.at<float>(vpoints[i].y, vpoints[i].x);
			Tmp(0, 0) = depthVal * (vpoints[i].x + screenRoi.x - cx)*1.0 / fx;
			Tmp(1, 0) = depthVal * (vpoints[i].y + screenRoi.y - cy)*1.0 / fy;
			Tmp(2, 0) = depthVal;
			PointInCol = DepToColR*Tmp + DepToColT;
			source->points[i].x = PointInCol(0, 0);
			source->points[i].y = PointInCol(1, 0);
			source->points[i].z = PointInCol(2, 0);
		}

		size_t iterations = 200;
		//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setEuclideanFitnessEpsilon(1e-8);
		//icp.setTransformationEpsilon(1e-6);
		//icp.setMaximumIterations(iterations);
		icp.setInputSource(source);
		icp.setInputTarget(target);
		icp.align(*result);
		Eigen::Matrix4f transformation = icp.getFinalTransformation(); // Obtain the transformation that aligned source to target

		cout << "result: " << endl;
		for (size_t ii = 0; ii < result->points.size(); ++ii)
			cout << result->points[ii].x << " " << result->points[ii].y << " " << result->points[ii].z << endl;

		Eigen::Matrix<double, 3, 3> RR;
		Eigen::Matrix<double, 3, 3> R_inv;
		Eigen::Matrix<double, 3, 1> TT;
		RR << transformation(0, 0), transformation(0, 1), transformation(0, 2),
			transformation(1, 0), transformation(1, 1), transformation(1, 2),
			transformation(2, 0), transformation(2, 1), transformation(2, 2);
		R_inv = RR.inverse();
		TT << transformation(0, 3), transformation(1, 3), transformation(2, 3);
		Eigen::Matrix<double, 3, 5> Corner;
		Eigen::Matrix<double, 3, 1> Point;
		for (size_t ii = 0; ii < target->points.size(); ++ii)
		{
			Point << target->points[ii].x, target->points[ii].y, target->points[ii].z;
			Point = R_inv * (Point - TT);
			Corner.col(ii) = Point;
		}
		Eigen::Matrix<double, 3, 5> CornerInImage;
		for (size_t ii = 0; ii < target->points.size(); ++ii)
		{
			CornerInImage.col(ii) = DepToColR.inverse()*(Corner.col(ii) - DepToColT);
			CornerInImage(0, ii) = CornerInImage(0, ii)*fx / CornerInImage(2, ii) +cx -screenRoi.x;
			CornerInImage(1, ii) = CornerInImage(1, ii)*fy / CornerInImage(2, ii) + cy - screenRoi.y;
			circle(foreground, Point2f(CornerInImage(0, ii), CornerInImage(1, ii)), 5, Scalar(0, 0, 255), -1);
		}
		cout << CornerInImage << endl;
			
    }
	imshow("foreground", foreground);
	waitKey(1);
}
/*
*function: refine detection verticals according to real model
*note    :
*/
void ProjectorCamera::clockwiseContour(vector<Point2f>& verticals)
{
    if(verticals.size()<3) return;

    Mat allPoints(verticals);
    Scalar center = mean(verticals);
    Point2f O(center.val[0], center.val[1]);
    Point2f a = verticals[0] - O;//o->a
    Point2f b = verticals[1] - O;//o->a

    float hint = a.x*b.y - b.x*a.y;

    if(hint < 0) reverse(verticals.begin(),verticals.end());
}

void ProjectorCamera::clockwiseContour(vector<Point>& verticals)
{
    if(verticals.size()<3) return;

    Mat allPoints(verticals);
    Scalar center = mean(verticals);
    Point2f O(center.val[0], center.val[1]);
    Point2f x1(verticals[0].x, verticals[0].y);
    Point2f x2(verticals[1].x, verticals[1].y);
    Point2f a = x1 - O;//o->a
    Point2f b = x2 - O;//o->a
	
    float hint = a.x*b.y - b.x*a.y;

    if(hint < 0) reverse(verticals.begin(),verticals.end());
}
/*
*function: refine detection verticals according to real model
*note    :
*/
void ProjectorCamera::refineVerticals(vector<Point3f>& verticals)
{
    if(verticals.size() != 4) return;

    double realDiagonal = SALVER_WIDTH_MM * sqrt(2.0);

    for(int ind=0;ind<2;ind++)
    {
        double detectDiagonal = calPointDistance(verticals[ind],verticals[ind+2]);
        double bias = (detectDiagonal - realDiagonal) / 2;

        Point3f unitVector = verticals[ind+2] - verticals[ind];
        double L = sqrt(unitVector.x * unitVector.x + unitVector.y * unitVector.y + unitVector.z * unitVector.z);
        unitVector.x = unitVector.x / L;
        unitVector.y = unitVector.y / L;
        unitVector.z = unitVector.z / L;

        verticals[ind] = verticals[ind] + bias * unitVector;
        verticals[ind+2] = verticals[ind+2] - bias * unitVector;
        //LOGD("refineVerticals be%f af%f",detectDiagonal,calPointDistance(verticals[ind],verticals[ind+2]));
    }
}
/*
*function: detection finger from foreground image
*note    :
*/
void ProjectorCamera::pointCamToPro(vector<Point2f>& vp_cam, vector<float>& v_depth,vector<Point3f>& vp_pro)
{
	//undistored points in image
	vector<Point2f> point_in_camera_undised(vp_cam);
	undistortPoints(vp_cam, point_in_camera_undised, camera_KK, camera_dis, Mat::eye(3, 3, CV_64F), camera_KK);

	//image plane-> camera axis->projector axis
	vp_pro.clear();
	for (int ind = 0; ind<point_in_camera_undised.size(); ind++)
	{
		//image plane-> camera axis
		Point2f temPoint = point_in_camera_undised[ind];
		double cx = camera_KK.at<double>(0, 2);
		double cy = camera_KK.at<double>(1, 2);
		double fx = camera_KK.at<double>(0, 0);
		double fy = camera_KK.at<double>(1, 1);
		double Zc = v_depth[ind];
		double Xc = Zc * (temPoint.x + screenRoi.x - cx) / fx;
		double Yc = Zc * (temPoint.y + screenRoi.y - cy) / fy;
		//LOGD("stereo projection in the air proAxis,Xc%f Yc%f Zc%f", Xc, Yc, Zc);
		//camera axis->projector axis
		Point3d camAxis(Xc, Yc, Zc);

		Mat out = camToProR * Mat(camAxis) + camToProT;
		Point3f proAxis;
		double pfx = project_KK.at<double>(0, 0);
		double pfy = project_KK.at<double>(1, 1);
		double pcx = project_KK.at<double>(0, 2);
		double pcy = project_KK.at<double>(1, 2);

		//proAxis.x = -out.at<double>(0,0);
		proAxis.x = (0) * out.at<double>(2, 0) / pfx - out.at<double>(0, 0);
		proAxis.y = (0) * out.at<double>(2, 0) / pfy + out.at<double>(1, 0);
		//proAxis.y = out.at<double>(1,0);
		proAxis.z = out.at<double>(2, 0);
		vp_pro.push_back(proAxis);
		//LOGD("stereo projection in the air proAxis,fx%f fy%f cx%f cy%f bias%f",pfx,pfy,pcx,pcy,(pcx) * out.at<double>(2,0) / pfy);
	}
		cout<<"finished"<<endl;
}

/*
*function: detection finger from foreground image
*note    :
*/
void ProjectorCamera::pointCamToPro(vector<Point2f>& vp_cam, vector<float>& v_depth,vector<Point2f>& vp_pro)
{

    //undistored points in image
    vector<Point2f> point_in_camera_undised;
	undistortPoints(vp_cam, point_in_camera_undised, camera_KK, camera_dis, Mat::eye(3, 3, CV_64F), camera_KK);

    //LOGD("stereo projection in the air point_in_camera_undised size : %d" , (int)point_in_camera_undised.size());
	//image plane-> camera axis->projector axis
    vp_pro.clear();
    for(int ind=0;ind<point_in_camera_undised.size();ind++)
    {
        //image plane-> camera axis
        Point2f temPoint = point_in_camera_undised[ind];
        double cx = camera_KK.at<double>(0,2);
        double cy = camera_KK.at<double>(1,2);
        double fx = camera_KK.at<double>(0,0);
        double fy = camera_KK.at<double>(1,1);
        double Zc = v_depth[ind];
        double Xc = Zc * (temPoint.x + screenRoi.x - cx) / fx;
		double Yc = Zc * (temPoint.y + screenRoi.y - cy) / fy;

        //LOGD("pointCamToPro z: xi%f yi%f xc%f yc%f zc%f", temPoint.x + screenRoi.x,
        //temPoint.y + screenRoi.y,float(Xc),float(Yc),float(Zc));
        //LOGD("pointCamToPro z: xi%f yi%f xc%f yc%f zc%f", temPoint.x + screenRoi.x,
        //       temPoint.y + screenRoi.y,float(Xc),float(Yc),float(Zc));
        //camera axis->projector axis
        Point3f camAxis;
        camAxis.x = Xc;
        camAxis.y = Yc;
        camAxis.z = Zc;

        vector<cv::Point2f> imagePoints2;
        vector<cv::Point3f> v_Rchess_in_world;
        v_Rchess_in_world.push_back(camAxis);
        projectPoints(cv::Mat(v_Rchess_in_world), camToProR, camToProT, project_KK,  // project
		project_dis, imagePoints2);
		imagePoints2[0].x = 1280 - imagePoints2[0].x;
		imagePoints2[0].y = 720 - imagePoints2[0].y;
        vp_pro.push_back(imagePoints2[0]);
        //Mat proAxis = camToProR * Mat(camAxis) + camToProT;
        //Mat proImag = project_KK * proAxis;

        //Point2f out;
        //out.x = 1280 - proImag.at<double>(0,0) / proImag.at<double>(2,0);
        //out.y = 720 - proImag.at<double>(0,1) / proImag.at<double>(2,0);
        //vp_pro.push_back(out);
    }
}

/*
*function: detection finger from foreground image
*note    :
*/
void ProjectorCamera::findFinger()
{    
	//LOGF("Processing flow : %s", "findFinger start");
	//clear last touch information
	VtouchPoints vtouchPoints_old(vtouchPoints);  //将之前的触碰信息存下来。
	vtouchPoints.clear();  //清除vtouchPoints,以存储本次的触碰信息。
	if (foreground_copy.empty()) cvtColor(foreground, foreground_copy, 8); //前景图存到foreground_copy中，备使用。
	Mat foreShow;
	cvtColor(foreground, foreShow, 8);
	for (int ind = 0; ind < objects.size(); ind++)
	{
		//if (objects[ind].maxdepth < 50 || !histcontinue(objects[ind].object)) continue;

	    //get DP points of the object
	    HandContour temHand;
		approxPolyDP(Mat(objects[ind].contour), temHand.approxCurve, 3, true);

        //手臂与边界处的交点
        int crossNum(0);
        for (int ind = 0; ind < temHand.approxCurve.size(); ind++)
		{
		    Point temPt = temHand.approxCurve[ind];
			if (temPt.x < 10 || depthImg.cols - temPt.x < 10||
				temPt.y < 10 || depthImg.rows - temPt.y < 10)
			{
				crossNum++;
			}
		}
		LOGD("findFinger frameID%d crossNum%d\n",frameID,crossNum);
		if(crossNum<2) continue;  //如果少于两个交点，则处理下一帧。
		//cout << "crossNum: " << crossNum << endl;

		temHand.approxCurve.clear();
		approxPolyDP(Mat(objects[ind].contour), temHand.approxCurve, 10, true);
		//get convex hull
		vector<int> hull;
		convexHull(Mat(temHand.approxCurve), hull, false, false);
        //if(hull.size() < 2) continue;

 		vector<Vec4i> defect;
 		if(temHand.approxCurve.size() > 3) convexityDefects(Mat(temHand.approxCurve), hull, defect);
 		Mat result;
 		CVTIME distanceTransformTime;
 		distanceTransform(bincp, result,  CV_DIST_L2, 3);
 		//LOGD("stereo projection distanceTransformTime: %f frameID%d",distanceTransformTime.getClock(), frameID);
 		vector<int> new_counter_ind;
 		vector<Point> new_conter_pt(temHand.approxCurve);
 		
 		maxVal = -1;
 		maxLoc = Point(-1, -1);
		if( new_conter_pt.size() > 1)
		{
			Rect contRect = boundingRect(new_conter_pt);
			int bound_height = 80;
			Rect boundRect = Rect(contRect.x, contRect.y, contRect.width, contRect.y + bound_height > screenRoi.height ? screenRoi.height - contRect.y : bound_height);
			rectangle(foreShow, boundRect, COLOR_RED, 2);
			for(int i = 0; i < new_conter_pt.size(); i++)
			//circle(foreground_copy, new_conter_pt[i], 3, COLOR_RED, 2);
			minMaxLoc(result(boundRect), &minVal, &maxVal, &minLoc, &maxLoc);

			maxLoc.x += boundRect.x;
			maxLoc.y += boundRect.y;
			circle(foreShow, maxLoc, maxVal, COLOR_BLUE, 2);
			//cout << endl << endl;
			//cout << "frameId: " << frameID << " maxLoc: " <<maxLoc << endl;
		}

		//string name = "/sdcard/aaa_" + intTostring(frameID) + ".jpg";
        //imwrite(name,foreground_copy);
        //LOGD("vtouchPoints_old, id %d x %d y %d val %f",frameID,maxLoc.x,maxLoc.y,maxVal);

		// assemble point set of convex hull
		vector<Point> hullPoints, hullforfinger;
		for (int j = 0; j < hull.size(); j++)
		{
			//index is convex point
			int index = hull[j], leftind(0), rightind(0);
			int Point_x = temHand.approxCurve[index].x;
			int Point_y = temHand.approxCurve[index].y;

			//boarder skip
			if (Point_x + 10 >= foreground.cols || Point_x - 10 <= 0 ||
			    Point_y + 10 >= foreground.rows || Point_y - 10 <= 0)
				continue;

			int radius = 12;
			Rect figerRec = Rect(Point_x - radius, Point_y - radius, 2*radius, 2*radius);
			refineRect(figerRec, foreground);
			Mat fingerRoi = foreground(figerRec);

			//first condition
			double mindepth,maxdepth;
			minMaxLoc(fingerRoi, &mindepth, &maxdepth);
			//LOGD("nativeStart caught in findFinger maxdepth: %f", maxdepth);
			if (maxdepth > 50 && maxdepth < 750) continue;

			if (!histcontinue(fingerRoi)) continue;

			//second condition
			float angle(0);
			if (index == 0)
			{
				leftind = temHand.approxCurve.size() - 1;
				rightind = index + 1;
			}
			else if (index == temHand.approxCurve.size() - 1)
			{
				leftind = index - 1;
				rightind = 0;
			}
			else
			{
				leftind = index - 1;
				rightind = index + 1;
			}

			angle = caclAngle(temHand.approxCurve[leftind], temHand.approxCurve[index],
			                  temHand.approxCurve[rightind]);
			LOGD("nativeStart caught in findFinger angle: %f", angle);			
            if (angle > 70) continue;
							
			float foresum(0), backsum(0);
			int   forenum(0), backnum(0);
			Point2f fingerTip(0,0);
			for (int row = Point_y - 5; row < Point_y + 5; row++)
			{
				float * depthPt = (float*)depthImg.row(row).data;
				float * foregPt = (float*)foreground.row(row).data;
				for (int col = Point_x - 5; col < Point_x + 5; col++)
				{
					if (*(depthPt + col) < 0.1)
					{
						fingerTip += Point2f(col,row);
						continue;
					}

					if (*(foregPt + col) > 0)
					{
						fingerTip += Point2f(col, row);
						foresum += *(depthPt + col);
						forenum++;
					}
					else
					{
						backsum += *(depthPt + col);
						backnum++;
					}
				}
			}

			if (forenum > 0 )
			{
				bfdis = -(foresum / forenum - backsum / backnum);
				fingerTip.x /= forenum;
				fingerTip.y /= forenum;
			}

			//third condition
			LOGD("nativeStart caught in findFinger bfdis: %f", bfdis);			
			if (bfdis > 8 || forenum<5) continue;

			//save touch points information
			TouchDetail touchDetail;
			touchDetail.tipPosition = fingerTip; //temHand.approxCurve[index];
			touchDetail.tipDepth    = averaImg.at<float>(touchDetail.tipPosition.y, touchDetail.tipPosition.x) + 8;
			touchDetail.bottomPosition = maxLoc;//fingerDirection(temHand.approxCurve[leftind], temHand.approxCurve[index], temHand.approxCurve[rightind]);
			touchDetail.bottomDepth = depthImg.at<float>(maxLoc);//averaImg.at<float>(touchDetail.bottomPosition.y, touchDetail.bottomPosition.x);
			touchDetail.getAngle();
			touchDetail.getOrien();
			//vtouchPoints.detailInfo.push_back(touchDetail);
			LOGD("nativeStart caught in findFinger vtouchPoints_old: %d %d", temHand.approxCurve[index].x, temHand.approxCurve[index].y);
			cout << "   " <<touchDetail.angle << endl;

            touchDetail.palmDepth = maxVal > 0 ? depthImg.at<float>(maxLoc) : maxVal;
        	touchDetail.palmCenter = maxLoc;
        	touchDetail.frameNum = frameID;
        	touchDetail.isReal = true;
        	vtouchPoints.detailInfo.push_back(touchDetail);

			vtouchPoints.dx = touchDetail.tipPosition.x - maxLoc.x;
			vtouchPoints.dy = touchDetail.tipPosition.y - maxLoc.y;
			circle(foreShow, touchDetail.tipPosition, 2, COLOR_BLUE, 2);
			circle(colorImg, touchDetail.tipPosition, 2, COLOR_BLUE, 2);
		}
	} // contour conditional

	imshow("foreShow", foreShow);
	imshow("colorImg", colorImg);
	waitKey(1);
	return;
    //lixing
    int size = vtouchPoints_old.detailInfo.size();
    if(size>0 && maxLoc.x>0)
    //LOGD("nativeStart caught in findFinger vtouchPoints_old, palmDepth%f d2%f maxVal%f",vtouchPoints_old.detailInfo[0].palmDepth,depthImg.at<float>(maxLoc),maxVal);	

	cout << "cursize: " << vtouchPoints.detailInfo.size()<< " size: " << size << " lastId: " << vtouchPoints_old.detailInfo[0].frameNum << " frameId: " << frameID << " lastDepth: " << vtouchPoints_old.detailInfo[0].palmDepth
		<< " depth: " << depthImg.at<float>(maxLoc) << " maxVal: " << maxVal << " dist: " << sqrt((maxLoc.x - vtouchPoints_old.detailInfo[0].palmCenter.x)*(maxLoc.x - vtouchPoints_old.detailInfo[0].palmCenter.x)
		+ (maxLoc.y - vtouchPoints_old.detailInfo[0].palmCenter.y)*(maxLoc.y - vtouchPoints_old.detailInfo[0].palmCenter.y)) << endl;
    if(size > 0
	   && vtouchPoints_old.detailInfo[0].frameNum == frameID - 1
	   && vtouchPoints_old.detailInfo[0].palmDepth > DEPTH_SURFACE
	   && maxVal > 0 && depthImg.at<float>(maxLoc) > DEPTH_SURFACE)
	{

	    float dist = sqrt((maxLoc.x - vtouchPoints_old.detailInfo[0].palmCenter.x)*(maxLoc.x - vtouchPoints_old.detailInfo[0].palmCenter.x)
					+(maxLoc.y - vtouchPoints_old.detailInfo[0].palmCenter.y)*(maxLoc.y - vtouchPoints_old.detailInfo[0].palmCenter.y));
		cout << "dist" << dist << endl;
        //LOGD("nativeStart caught in findFinger vtouchPoints_old, id%d dis %f size%d oldsize%d" ,frameID,dist,vtouchPoints.detailInfo.size(),size);
		if(dist > 1)
		{
			int curNum = vtouchPoints.detailInfo.size();
			if(curNum < size)
			{
				
			    for(int idx = 0; idx < size; idx++)
				{
					float deltaX = vtouchPoints.cameraPts[idx].x - vtouchPoints_old.detailInfo[idx].palmCenter.x;
					float deltaY = vtouchPoints.cameraPts[idx].y - vtouchPoints_old.detailInfo[idx].palmCenter.y;
					vtouchPoints.cameraPts.clear();
					vtouchPoints.clear();
					vtouchPoints.cameraPts.push_back(Point(maxLoc.x + vtouchPoints_old.dx, maxLoc.y + vtouchPoints_old.dy));
					TouchDetail tmpDetail = vtouchPoints_old.detailInfo[idx];
					tmpDetail.isReal = false;
					tmpDetail.frameNum = frameID;
					tmpDetail.palmCenter = maxLoc;
					tmpDetail.palmDepth = depthImg.at<float>(maxLoc);
					vtouchPoints.detailInfo.push_back(tmpDetail);
					vtouchPoints.dx = vtouchPoints_old.dx;
					vtouchPoints.dy = vtouchPoints_old.dy;
					circle(foreground_copy, vtouchPoints.cameraPts[0], 2, COLOR_DARK_GREEN, 2);
					//LOGD("nativeStart caught in findFinger vtouchPoints_old: id%d curnum%f x%f y%f",frameID,curNum,
					//vtouchPoints.cameraPts[0].x,vtouchPoints.cameraPts[0].y);
					cout << "maxLoc: "<<maxLoc<<" cameraPts: " << vtouchPoints.cameraPts[0] << endl;
				}
			}
		}
    }
	
	imshow("foreground_copy", foreground_copy);
	waitKey(1);
	//transform position from camera to projector
	transAxisCameraToPro();
}

/*
*function: detection in the ari gesture from foreground image
*note    : by zhang bo
*/
void ProjectorCamera::findInAirGesture(){
	int objectssize = objects.size();
	if (!objectssize)
		return;

	//将前景转化为3通道,方便显示
	vector<Mat> foregrounds;
	for (int i = 0; i < 3; i++)
		foregrounds.push_back(foreground);
	cv::merge(foregrounds, foreground_copy);

	for (int i = 0; i < objects.size(); i++){
		//min object area
		int cArea = objects[i].cArea;
		if (cArea < 1000 || cArea > 40000) continue;

		Mat contourMat = Mat(objects[i].contour);

		vector<Point> approxCurve;
		//get DP points of the object
		approxPolyDP(contourMat, approxCurve, 10, true);

		vector< vector<Point> > debugContourV;
		debugContourV.push_back(approxCurve);

		//get convex hull
		vector<int> hull;
		convexHull(Mat(approxCurve), hull, false, false);

		// find convexity defects
		vector<ConvexityDefect> convexDefects;
		findConvexityDefects(approxCurve, hull, convexDefects);


		// assemble point set of convex hull
		vector<Point> hullPoints;
		for (int k = 0; k < hull.size(); k++){
			int curveIndex = hull[k];
			Point p = approxCurve[curveIndex];
			hullPoints.push_back(p);
		}
		// area of hull and curve
		double hullArea = contourArea(Mat(hullPoints));//凸包点
		double curveArea = contourArea(Mat(approxCurve));//凹包点+凸包点
		double handRatio = curveArea / hullArea;

		//calculate the centers of the likely hand

		Point center_point = Size(0, 0); int pointnum = 0;int nedgenum = 0;
		for (int i = 0; i < approxCurve.size(); i++){
			//靠近边缘的凸包点剔除
			if (approxCurve[i].x < 5 || approxCurve[i].y < 5 || approxCurve[i].x>foreground_copy.cols - 5 || approxCurve[i].y>foreground_copy.rows - 5)
				nedgenum++;
			pointnum++;
			center_point = CalculateSumofPoints(center_point, approxCurve[i]);
		}
		if (pointnum <= 3 /*|| nedgenum >= 3*/) continue;
		center_point.x = center_point.x / pointnum;
		center_point.y = center_point.y / pointnum;

		//太过边缘的中心点剔除
		if (center_point.x < 20 || center_point.y < 10 || center_point.x > foreground_copy.cols - 20 || center_point.y > foreground_copy.rows - 10 ) continue;

		//push hand centers into vector
		if (center_point.x || center_point.y){
			rectangle(foreground_copy, Rect(center_point.x - cube_size / 2, center_point.y - cube_size / 2, cube_size, cube_size), Scalar(0, 0, 255), 3, 8, 0);
			pointsInLastFrames.push_back(center_point);//推入新的帧
			if (pointsInLastFrames.size() > trajFrameNum)
				pointsInLastFrames.erase(pointsInLastFrames.begin());//清除第一帧
		}

		//rectified by zb：将凹凸包点同时判定，同时修改angle的约束
		int nlittleangle = 0;
		for (int j = 0; j < approxCurve.size(); j++)
		{
			//所有凹凸点的index
			int index = j, leftind(0), rightind(0);

			//所有凹凸点构成的角度
			float angle(0);
			if (index == 0)
			{
				leftind = approxCurve.size() - 1;
				rightind = index + 1;
			}
			else if (index == approxCurve.size() - 1)
			{
				leftind = index - 1;
				rightind = 0;
			}
			else
			{
				leftind = index - 1;
				rightind = index + 1;
			}
			//将靠近边缘的角度判定剔除
			if (approxCurve[index].x < 10 || approxCurve[index].y < 10 || approxCurve[index].x>foreground_copy.cols - 10 || approxCurve[index].y>foreground_copy.rows - 10)
				continue;
			angle = caclAngle(approxCurve[leftind], approxCurve[index], approxCurve[rightind]);

			if (angle < 75)//
				nlittleangle++;
		}
		//1.hand grasp state
		int approxCurvesize = approxCurve.size(); int hullPointssize = hullPoints.size();
		int maxdistance_x = abs(pointsInLastFrames[0].x - pointsInLastFrames[pointsInLastFrames.size() - 1].x);
		bool openstate = isHandGraspOrOpen(handRatio, approxCurvesize, hullPointssize, nlittleangle, maxdistance_x, center_point);

		//2.hand wave state
		int wavestate = handWaveState(pointsInLastFrames);

		//in air gesture management
		inAirGestureManage(openstate, wavestate);
	}
}

/*
*function:fill the holes in depth image
*note    :
*/
void ProjectorCamera::fillDepthImageHole(Mat& object)
{
    for (int row = 0; row < object.rows; row++)
	{
		float * pt = (float *)object.row(row).data;
		for (int col = 0; col < object.cols; col++)
		{
			if (*(pt + col) > nearCameraThresh)
			{
				vector<float> valid; float val(0);
				for (int srow = max(0, row - 1); srow < min(object.rows, row + 2); srow++)
				{
					float* spt = (float *)object.row(srow).data;
					for (int scol = max(0, col - 1); scol < min(object.cols, col + 2); scol++)
					{
						if (*(spt + scol) < nearCameraThresh) valid.push_back(*(spt + scol));
					}
				}

				if (valid.empty()) *(pt + col) = 44;
				else
				{
					val = std::accumulate(valid.begin(), valid.end(), 0);
					*(pt + col) = val / valid.size();
				}
			}//end thresh condition
		}//end cols
	}//end rows
}

/*
*function:translate point location form camera to Projector
*note    :
*/
void ProjectorCamera::transAxisCameraToPro()
{
    if (vtouchPoints.detailInfo.size() > 0)
	{
	    //sort points for tracking
	    //sort(vtouchPoints.detailInfo.begin(),vtouchPoints.detailInfo.end(),cmpTouchDetail);

        //translate points
	    vtouchPoints.cameraPts.clear();
	    vector<float> v_depth;
	    for(int ind=0;ind<vtouchPoints.detailInfo.size();ind++)
	    {
	        vtouchPoints.cameraPts.push_back(vtouchPoints.detailInfo[ind].tipPosition);
	        v_depth.push_back(vtouchPoints.detailInfo[ind].tipDepth);
	    }
		perspectiveTransform(vtouchPoints.cameraPts, vtouchPoints.projecPts, h);

		//add offset
		for (int ind = 0; ind < vtouchPoints.projecPts.size(); ind++)
		{
			vtouchPoints.projecPts[ind].x += offset_x + offset_init_x;
			vtouchPoints.projecPts[ind].x = 1280 - vtouchPoints.projecPts[ind].x + 5;
			vtouchPoints.projecPts[ind].y += offset_y + offset_init_y;
			vtouchPoints.projecPts[ind].y = 720 - vtouchPoints.projecPts[ind].y - 5;
		}
		//pointCamToPro(vtouchPoints.cameraPts, v_depth, vtouchPoints.projecPts);
	}
	else
	{
	    //LOGD1("nativeStart caught vtouchPoints.projecPts notouch");
	}

}

/*
*function: calculate object average depth
*note    :
*/
float ProjectorCamera::contourAverageDepth(vector<Point>& contour)
{
	Rect contRect = boundingRect(contour);
	int pointCount(0);
	float depthSum(0);
	for (int row = contRect.y; row < contRect.y + contRect.height; row++)
	{
		for (int col = contRect.x; col < contRect.x + contRect.width; col++)
		{
			if (pointPolygonTest(contour, Point(col, row), 1) >= 0 &&
				foreground.at<float>(row, col) > 0)
			{
				pointCount++;
				depthSum += foreground.at<float>(row, col);
			}

		}
	}

	return depthSum / pointCount;
}

/*
*function: calculate angle constructed by three points
*note    :
*/
float  ProjectorCamera::caclAngle(Point& left, Point& center, Point& right)
{
	float angle(0);
	Point lc = left - center;
	Point rc = right - center;

	float cos_angle = (lc.x * rc.x + lc.y*rc.y) / (sqrtf(lc.x*lc.x + lc.y*lc.y) * sqrtf(rc.x*rc.x + rc.y *rc.y));

	angle = acos(cos_angle);

	angle *= 180 / M_PI;
	return angle;
}

/*
*function:calculate the direction of finger tip
*note    :
*/
Point2f ProjectorCamera::fingerDirection(Point2f L, Point2f M, Point2f R)
{
	//make sure L nearer than R
	if (PointDistence(L, M) > PointDistence(R, M))
	{
		Point2f tem = L;
		L = R;
		R = tem;
	}

	//calculate the L pojection point in MR
	float dis = PointDistence(L, M);
	Point2f Fdirection = R - M;
	float normtem = norm(Fdirection);
	Fdirection.x /= normtem;
	Fdirection.y /= normtem;
	Point2f O = M + dis * Fdirection;
	O = (O + L)*0.5;

	return O;
}


/*
*function:calculate distance between two points
*note
*/
//float ProjectorCamera::calPointDistance(Point& pointA, Point& pointB)
//{
//	return sqrt((float)((pointA.x - pointB.x)*(pointA.x - pointB.x) + (pointA.y - pointB.y)*(pointA.y - pointB.y)));
//}

float ProjectorCamera::calPointDistance(Point2f pointA, Point2f pointB)
{
	return sqrt((float)((pointA.x - pointB.x)*(pointA.x - pointB.x) + (pointA.y - pointB.y)*(pointA.y - pointB.y)));
}

float ProjectorCamera::calPointDistance(Point3f pointA, Point3f pointB)
{
	return sqrt((float)((pointA.x - pointB.x)*(pointA.x - pointB.x) +
	(pointA.y - pointB.y)*(pointA.y - pointB.y) + (pointA.z - pointB.z)*(pointA.z - pointB.z)));
}


/*
*function:make sure rectangle do not beyond corresponding image
*note    :
*/
void ProjectorCamera::refineRect(Rect& rec, Mat image)
{
	if (rec.x < 0) rec.x = 0;
	if (rec.y < 0) rec.y = 0;

	if (rec.x + rec.width  > image.cols) rec.width = image.cols - rec.x;
	if (rec.y + rec.height > image.rows) rec.height = image.rows - rec.y;
}

/*
*function:calculate the distance between two points
*note    :
*/
float PointDistence(Point2f& p1, Point2f& p2)
{
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;

	return sqrt(dx*dx + dy*dy);
}


/*
*function:get finger tip point direction
*note    :0-90
*/
void TouchDetail::getDirection()
{
	direction = tipPosition - bottomPosition;
	float normtem = norm(direction);
	direction.x /= normtem;
	direction.y /= normtem;
};

/*
*function:get finger tip point angle
*note    :0-90
*/
void TouchDetail::getAngle()
{
	angle = atan2(abs(tipDepth - bottomDepth), PointDistence(tipPosition,bottomPosition)*0.6) / M_PI * 180;

};

/*
*function:get finger tip point orientation
*note    :0-360,normal Y+ is 0 in android screen
*/
void TouchDetail::getOrien()
{
    //calculate the angle between Y axis
    orien = atan2(abs(tipPosition.x - bottomPosition.x), abs(tipPosition.y - bottomPosition.y)) / M_PI * 180;

    //convert to 0-360
    if(tipPosition.x >= bottomPosition.x)
    {
        //2 quadrant
        if(tipPosition.y >= bottomPosition.y)
        {
            orien = 360 - orien;
        }
        //3 quadrant
        else
        {
            orien = 180 + orien;
        }
    }
    else
    {
        //1 quadrant
        if(tipPosition.y >= bottomPosition.y)
        {
            //orien = orien;
        }
        //4 quadrant
        else
        {
            orien = 180 - orien;
        }
    }

    orien += 180;
    if(orien > 360) orien -= 360;
}

bool ProjectorCamera::isHandGraspOrOpen(double& handRatio, int& approxCurveSize, int& hullsize, int& littleangle, int& distance_x,Point& centerpoint){
	ncount_totalimg++;//hand grasp state
	ncount_totalimg_open++;//hand open state
	bool grapestate = GRASPFAITTURE;
	bool openstate = OPENFAITURE;

	//单帧判定的结果
	bool hullstate = ((approxCurveSize - hullsize) >= 1 && (approxCurveSize - hullsize) <= 4) && (hullsize >= 5 && hullsize <= 7) && (approxCurveSize <= 10 && approxCurveSize >= 6);
	bool pointstate = (centerpoint.x > 120 && centerpoint.x < screenRoi.width- 120 && centerpoint.y >20 && centerpoint.y < screenRoi.height-20);
	//hand grasp state
	if (/*handRatio > 0.7&&*/ /*handRatio<0.95&&*/ !littleangle && hullstate && pointstate){
		ncount_grasp++;
	}
	//hand open state------------------new
	else if (littleangle >= 2 && approxCurveSize >= 10 && approxCurveSize - hullsize >= 3 && pointstate){
		ncount_open++;
	}

	//多帧给出最后结果，hand grasp state output
	if (ncount_totalimg == trajFrameNum){//make a decision every trajFrameNum
		if (ncount_grasp >= trajFrameNum-1 && distance_x < 40 ){//first frame is no grasping,last frame is grasping.
			grapestate = GRASPSUCCESS;
		}
		ncount_grasp = 0;
		ncount_totalimg = 0;
	}
	handgraspstates_temp.push_back(grapestate);

	//hand open state output----------new
	if (grapestate == GRASPSUCCESS){
		ncount_open = 0;
		ncount_totalimg_open = 0;
	}
	else if (grapestate == GRASPFAITTURE){
		if (ncount_totalimg_open == trajFrameNum*ntimes){
			if (ncount_open >= trajFrameNum*(ntimes -1)){
				openstate = OPENSUCCESS;
			}
		ncount_open = 0;
		ncount_totalimg_open = 0;
		}
	}
	return openstate;
}


int ProjectorCamera::handWaveState(vector<Point>& pointsInLastFrames){

	int frameNum = pointsInLastFrames.size();
	if (frameNum != trajFrameNum)
		return NOWAVE;
	//检测xy方向为升序或者降序（升为1，降为-1）
	int isSortAscending_x = pointsInLastFrames[0].x <= pointsInLastFrames[1].x ? 1 : -1;//x:1-->right,-1-->left.
	int isSortAscending_y = pointsInLastFrames[0].y <= pointsInLastFrames[1].y ? 1 : -1;//y:1-->dowm,-1-->up.
	bool issorted_x = true, issorted_y = true;//排序标志位
	//检查x方向有序性
	for (int nsortx = 0; nsortx < pointsInLastFrames.size() - 1; nsortx++){
		if (isSortAscending_x*(pointsInLastFrames[nsortx].x - pointsInLastFrames[nsortx + 1].x) > 0){
			issorted_x = false;
			break;
		}
	}
	float maxdistace_x = abs(pointsInLastFrames[0].x - pointsInLastFrames[frameNum - 1].x);
	float maxdistace_y = abs(pointsInLastFrames[0].y - pointsInLastFrames[frameNum - 1].y);

	//hand wave in x derection
	if (/*!issorted_y &&*/ issorted_x && maxdistace_x > 175 && maxdistace_y < 60){
		//TODO: hand wave detection in x direction
		if (isSortAscending_x == 1 && pointsInLastFrames[0].x <= screenRoi.width / 2 && pointsInLastFrames[trajFrameNum-1].x >= screenRoi.width / 2){//right
			return WAVERIGHT;
		}
		else if (isSortAscending_x == -1 && pointsInLastFrames[trajFrameNum - 1].x <= screenRoi.width / 2 && pointsInLastFrames[0].x >= screenRoi.width / 2){//left
			return WAVELEFT;
		}
	}
	return NOWAVE;
}

void ProjectorCamera::findConvexityDefects(vector<Point>& contour, vector<int>& hull, vector<ConvexityDefect>& convexDefects)
{
	if (hull.size() > 0 && contour.size() > 0)
	{
		CvSeq* contourPoints;
		CvSeq* defects;
		CvMemStorage* storage;
		CvMemStorage* strDefects;
		CvMemStorage* contourStr;
		CvConvexityDefect *defectArray = 0;

		strDefects = cvCreateMemStorage();
		defects = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), strDefects);

		//We transform our vector<Point> into a CvSeq* object of CvPoint.
		contourStr = cvCreateMemStorage();
		contourPoints = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), contourStr);
		for (int i = 0; i < (int)contour.size(); i++) {
			CvPoint cp = { contour[i].x, contour[i].y };
			cvSeqPush(contourPoints, &cp);
		}

		//Now, we do the same thing with the hull index
		int count = (int)hull.size();
		//int hullK[count];
		int* hullK = (int*)malloc(count*sizeof(int));
		for (int i = 0; i < count; i++) { hullK[i] = hull.at(i); }
		CvMat hullMat = cvMat(1, count, CV_32SC1, hullK);

		// calculate convexity defects
		storage = cvCreateMemStorage(0);
		defects = cvConvexityDefects(contourPoints, &hullMat, storage);
		defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*defects->total);
		cvCvtSeqToArray(defects, defectArray, CV_WHOLE_SEQ);
		//printf("DefectArray %i %i\n",defectArray->end->x, defectArray->end->y);

		//We store defects points in the convexDefects parameter.
		for (int i = 0; i<defects->total; i++){
			ConvexityDefect def;
			def.start = Point(defectArray[i].start->x, defectArray[i].start->y);
			def.end = Point(defectArray[i].end->x, defectArray[i].end->y);
			def.depth_point = Point(defectArray[i].depth_point->x, defectArray[i].depth_point->y);
			def.depth = defectArray[i].depth;
			convexDefects.push_back(def);
		}

		// release memory
		cvReleaseMemStorage(&contourStr);
		cvReleaseMemStorage(&strDefects);
		cvReleaseMemStorage(&storage);
	}
}

Point ProjectorCamera::CalculateSumofPoints(Point pointA, Point pointB){
	return Size(pointA.x + pointB.x, pointA.y + pointB.y);
}

void ProjectorCamera::convetFloatToChar(Mat& src, Mat& dst)
{
	dst = Mat(src.rows, src.cols, CV_8UC1, Scalar(0));

	if (src.isContinuous() && dst.isContinuous())
	{
		float * srcPt = (float *)src.data;
		uchar * dstPt = dst.data;
		for (int ind = 0; ind<src.rows * src.cols; ind++)
		{
			if ((*srcPt) > 0) *dstPt = 1;
			srcPt++;
			dstPt++;
		}
	}
	else
	{
		for (int row = 0; row<src.rows; row++)
		{
			float * srcPt = (float *)src.row(row).data;
			uchar * dstPt = dst.row(row).data;
			for (int col = 0; col<src.cols; col++)
			{
				if ((*srcPt) > 0) *dstPt = 1;
				srcPt++;
				dstPt++;
			}
		}
	}
}

void ProjectorCamera::inAirGestureManage(bool& openstate, int& handwave){

	//1.hand grasp management
	if (handgraspstates_temp.size() == trajFrameNum){
		//if (handgraspstates_temp.back() == GRASPSUCCESS)
		//	cout << "hand grasped!!!" << endl;
		graspstate_temp = handgraspstates_temp.back();
		handgraspstates_temp.clear();
	}
	if (graspstate_temp == GRASPSUCCESS&& !m_graspstate){
		lastframe_grasp = frameID;
		m_graspstate = true;
		graspstate_finnal = GRASPSUCCESS;
		return;
	}
	//else if (ncount_totalimg == trajFrameNum - 1 && ncount_grasp < trajFrameNum - 1){
	else if (frameID - lastframe_grasp > trajFrameNum*(ntimes+2)){
		m_graspstate = false;
		graspstate_finnal = GRASPFAITTURE;
	}
	else{
		graspstate_finnal = GRASPFAITTURE;
	}

	//2.hand open management
	if (openstate == OPENSUCCESS&&!m_openstate){
		m_openstate = true;
		openstate_finnal = OPENSUCCESS;
		lastframe = frameID;
		return;
	}
	else if (openstate == OPENSUCCESS && frameID - lastframe>=trajFrameNum*(ntimes+1)){
		m_openstate = false;
		openstate_finnal = OPENFAITURE;
	}
	else{
		openstate_finnal = OPENFAITURE;
	}

	//3.hand wave management
	if (1 == handwave&&m_wavestate != 1 ){
		//lastframe_wave = frameID;
		wavestate_finnal = WAVERIGHT;
		m_wavestate = 1;
		return;
	}
	else if (2 == handwave&&m_wavestate != 2 ){
		//lastframe_wave = frameID;
		wavestate_finnal = WAVELEFT;
		m_wavestate = 2;
		return;
	}
	//else if (frameID - lastframe_wave > trajFrameNum * (ntimes+1)){
	else if (0 == handwave){
		m_wavestate = 0;
		wavestate_finnal = NOWAVE;
	}
	else{
		wavestate_finnal = NOWAVE;
	}

}
