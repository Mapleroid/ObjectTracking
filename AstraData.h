#pragma once

#include <stdlib.h>  
#include <iostream>  
#include <string>  
#include "OpenNI.h"  
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
using namespace std;
using namespace cv;
using namespace openni;


class AstraData
{
public:
	AstraData();
	~AstraData();
	void init();
	void getData();

	void init_IR();
	void getData_IR();

	//OpenCV image  
	cv::Mat cvDepthImg8U;// for show
	cv::Mat cvRawImg16U;//for use
	cv::Mat cvBGRImg;
	cv::Mat cvIrImg;

private:
	//OpenNI2 image  
	VideoFrameRef oniDepthImg;
	VideoFrameRef oniColorImg;
	VideoFrameRef oniIrImg;

	Status result;
	VideoStream oniDepthStream;
	VideoStream oniColorStream;
	VideoStream oniIrStream;
	Device device;
};

