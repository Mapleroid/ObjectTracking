/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	idle.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       idle state processing file
*********************************************************************/

#ifndef _IDLE_
#define _IDLE_

//#include "state.h"
#include "role.h"

class Idle :public State
{
private:
	Idle();
	~Idle();

public:
	static State* getInstance();

public:
	void Excute(Role*);

	void dealIdelTouch(Role*);

	void dealSingleTouch(Role*);

	void dealOneHandDoubleTouch(Role*);

	void dealTwoHandDoubleTouch(Role*);
};
#endif