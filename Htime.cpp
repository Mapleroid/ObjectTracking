#include "htime.h"

/**************************C++ time function***********************/
htime::htime(void)
{
	running = false;
}


htime::~htime(void)
{
}

void htime::start()
{
	t1 = clock();
	running = true;
}
void htime::reset()
{
	t1=clock();
}
float htime::getClock()
{
	return ( clock()-t1 );
}

/**************************opencv time function***********************/
CVTIME::CVTIME()
{
	clock = cv::getTickCount();
}

CVTIME::~CVTIME()
{
}

void CVTIME::reset()
{
	clock = cv::getTickCount();
}

double CVTIME::getClock()
{
	clock = cv::getTickCount() - clock;
	return clock / cv::getTickFrequency() * 1000;
}