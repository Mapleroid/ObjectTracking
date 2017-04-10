/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	idle.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       idle state processing file
*********************************************************************/


#include "idle.h"
#include "singleActive.h"
#include "doubleOneHandActive.h"
#include "doubleTwoHandActive.h"
/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
Idle::Idle()
{
    setDescription("Idle");
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
Idle::~Idle()
{

}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
void Idle::Excute(Role* role)
{
	switch (role->getCurtTouchState())
	{
	case CURT_IDLE_TOUCH:
	{
		dealIdelTouch(role);
		break;
	}
	case CURT_SINGLE_TOUCH:
	{
		dealSingleTouch(role);
		break;
	}
	case CURT_1HAND2_TOUCH:
	{
		dealOneHandDoubleTouch(role);
		break;
	}
	case CURT_2HAND2_TOUCH:
	{
		dealTwoHandDoubleTouch(role);
		break;
	}
	default:
		break;
	}

	return;
}


/*!
@function
@abstract              deal current idle touch state
@discussion
@param
@result
*/
void Idle::dealIdelTouch(Role* role)
{
	return;
}

/*!
@function
@abstract              deal current single touch state
@discussion
@param
@result
*/
void Idle::dealSingleTouch(Role* role)
{
	//change state
	role->currentState = SingleActive::getInstance();

	//clear histroy data
	role->histTouchHands.clear();

	//set paras
	role->histTouchHands.clear();
	role->firstTouchId = role->curtTouchHands[0].frameId;
	role->lastTouchId = role->curtTouchHands[0].frameId;
	role->histTouchHands.push_back(role->curtTouchHands);

	return;
}

/*!
@function
@abstract              deal current one hand double touch state
@discussion
@param
@result
*/
void Idle::dealOneHandDoubleTouch(Role* role)
{
	//change state
	role->currentState = DoubleOneHandActive::getInstance();

	//clear histroy data
	role->histTouchHands.clear();

	//set paras
	role->firstTouchId = role->curtTouchHands[0].frameId;
	role->lastTouchId = role->curtTouchHands[0].frameId;
	role->histTouchHands.push_back(role->curtTouchHands);

	return;
}

/*!
@function
@abstract              deal current two hand double touch state
@discussion
@param
@result
*/
void Idle::dealTwoHandDoubleTouch(Role* role)
{
	//change state
	role->currentState = DoubleTwoHandActive::getInstance();

	//clear histroy data
	role->histTouchHands.clear();

	//set paras
	role->firstTouchId = role->curtTouchHands[0].frameId;
	role->lastTouchId = role->curtTouchHands[0].frameId;
	role->histTouchHands.push_back(role->curtTouchHands);

	return;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
State* Idle::getInstance()
{
	static Idle instance;
	return &instance;
}