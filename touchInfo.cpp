/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	touchInfo.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       touch information file
*********************************************************************/


#include "touchInfo.h"

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
TouchHand::TouchHand()
{
	frameId   = -1;
	palmDepth = -1;
	//radius in pix
	palmRadius = -1;
	//radius in mm
	realRadius = -1;
	palmCenter = Point(-1,-1);
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
TouchHand::~TouchHand()
{

}

/*!
@function
@abstract              get finger tip point angle
@discussion
@param
@result				   angle between touch finger and desk
*/
void TouchPoint::getAngle()
{
	angle = atan2(abs(tipDepth - bottomDepth),
		    norm(tipPosition -  bottomPosition)) / M_PI * 180;
};

/*!
@function
@abstract              get finger tip point direction
@discussion
@param
@result				   unit direction vector of touch finger
*/
void TouchPoint::getDirection()
{
	direction = tipPosition - bottomPosition;
	float normtem = norm(direction);
	direction.x /= normtem;
	direction.y /= normtem;
};

/*!
@function
@abstract              get finger tip point orientation
@discussion
@param
@result				   0-360,normal Y+ is 0 in android screen
*/
void TouchPoint::getOrien()
{
	//calculate the angle between Y axis
	orien = atan2(abs(tipPosition.x - bottomPosition.x), 
		          abs(tipPosition.y - bottomPosition.y)) / M_PI * 180;

	//convert to 0-360
	if (tipPosition.x >= bottomPosition.x)
	{
		//2 quadrant
		if (tipPosition.y >= bottomPosition.y)
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
		if (tipPosition.y >= bottomPosition.y)
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
	if (orien > 360) orien -= 360;
}