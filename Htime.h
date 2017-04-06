//#pragma once
#ifndef hTIME_MYTIME_CLASS
#define hTIME_MYTIME_CLASS

#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>

class htime
{
public:
	htime(void);
	~htime(void);
	bool running;
	clock_t t1;
	void reset();
	void start();
	float getClock();
};


class CVTIME
{
public:
	CVTIME();
	~CVTIME();

	int64 clock;
	void reset();
	double getClock();
};

#endif