#include "projectorCamera.h"
#include  "AstraData.h"


void mouse(int event, int x, int y, int flags, void*);
int  main(int argc, char* argv[])
{

	ProjectorCamera rsPeojection;
	rsPeojection.calibrationPerspective();
	//rsPeojection.screenRoi = Rect(30,0,300,200);

	AstraData astradata;
	astradata.init_IR();
	//astradata.init(); //astradata.cvBGRImg.empty()
	int flag=0;
	while (1)
	{
		astradata.getData_IR();
		if (astradata.cvRawImg16U.empty() || astradata.cvIrImg.empty())
			continue;
		Mat depthImg, colorImg, irImg;

		depthImg = astradata.cvRawImg16U.clone();
		//colorImg = astradata.cvBGRImg.clone();
		irImg = astradata.cvIrImg.clone();
		depthImg.convertTo(depthImg, CV_32F);
		//flip(colorImg, colorImg, 1);
		flip(irImg, irImg, 0);
		flip(depthImg, depthImg, 0);
		//Rect screenRoi   = Rect(80, 80, 400, 250);
		//depthImg = depthImg(screenRoi);
		//imshow("depthImg", depthImg);
		//waitKey(1);
		rsPeojection.processing(irImg, depthImg);
	}
	return 0;
}

/*
void mouse(int event, int x, int y, int flags, void*)
{

	if(event == EVENT_LBUTTONDOWN)
	{
		flag++;
		circle(colorImg, Point(x,y), 3, Scalar(0,0,255), -1);
	    cout<<x<<", "<<y<<", "<<depthImg.at<float>(y,x)<<endl;
		imshow("colorImg", colorImg);
	}
}
*/