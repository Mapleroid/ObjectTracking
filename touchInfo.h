/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	touchInfo.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       touch information file
*********************************************************************/


#ifndef _TOUCH_INFO_
#define _TOUCH_INFO_
#define _USE_MATH_DEFINES

#include <string>
#include <vector>
#include <Math.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class TouchPoint
{
public:
	//finger touch information in depth camera
	Point2f tipPosition, bottomPosition, direction, palmToTip;
	float   angle, orien;
	float   tipDepth, bottomDepth;	
	int     frameId;
	bool    isReal;


	//finger touch information in projector camera
	Point2f tipInPro;

public:
	TouchPoint() :angle(0), tipDepth(0), bottomDepth(0), orien(0){};
	void getDirection();
	void getAngle();
	void getOrien();
};

class TouchHand
{
public:
	//Hand touch information
	TouchHand();
	~TouchHand();

	float   palmDepth;
	double  palmRadius;
	Point   palmCenter;
	double  realRadius;

	int     frameId;
	vector<Point> approxCurve;
	vector<TouchPoint> touchPoints;
	
};
#endif