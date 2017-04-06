#include "AstraData.h"

void CheckOpenNIError(Status result, string status)
{
	if (result != STATUS_OK)
		cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}

AstraData::AstraData()
{
	result = STATUS_OK;
}


AstraData::~AstraData()
{
	//OpenNI2 destroy  
	oniDepthStream.destroy();
	oniColorStream.destroy();
	oniIrStream.destroy();
	device.close();
	OpenNI::shutdown();
}

void AstraData::init()
{
	// 1.initialize OpenNI2  
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	result = device.open(openni::ANY_DEVICE);

	// 2.create depth stream 
	result = oniDepthStream.create(device, openni::SENSOR_DEPTH);

	// 3.set depth video mode  
	VideoMode modeDepth;
	modeDepth.setResolution(640, 480);
	modeDepth.setFps(30);
	modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode(modeDepth);
	// start depth stream  
	result = oniDepthStream.start();

	result = oniColorStream.create(device, openni::SENSOR_COLOR);
	// set color video mode  
	VideoMode modeColor;
	modeColor.setResolution(640, 480);
	modeColor.setFps(30);
	modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
	oniColorStream.setVideoMode(modeColor);

	// 4.set depth and color imge registration mode  
	if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	// start color stream  
	result = oniColorStream.start();

	return;

}

void AstraData::init_IR()
{
	// 1.initialize OpenNI2  
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	result = device.open(openni::ANY_DEVICE);

	// 2.create depth stream 
	result = oniDepthStream.create(device, openni::SENSOR_DEPTH);

	// 3.set depth video mode  
	VideoMode modeDepth;
	modeDepth.setResolution(640, 480);
	modeDepth.setFps(30);
	modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode(modeDepth);
	// start depth stream  
	result = oniDepthStream.start();

	result = oniIrStream.create(device, openni::SENSOR_IR);
	// set color video mode  
	VideoMode modeIr;
	modeIr.setResolution(640, 480);
	modeIr.setFps(30);
	modeIr.setPixelFormat(PIXEL_FORMAT_GRAY16);
	oniIrStream.setVideoMode(modeIr);

	// 4.set depth and color imge registration mode  
	if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		//device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	// start color stream  
	result = oniIrStream.start();

	return;

}


void AstraData::getData()
{
	// read frame  
	if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK)
	{
		cv::Mat tempImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
		cvRawImg16U = tempImg16U.clone();
		//cv::imshow("cvRawImg16U", cvRawImg16U);
		cvRawImg16U.convertTo(cvDepthImg8U, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
		//cv::imshow("depth", cvDepthImg8U);
	}
	if (oniColorStream.readFrame(&oniColorImg) == STATUS_OK)
	{
		// convert data into OpenCV type  
		cv::Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());
		cv::cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
		//cv::imshow("image", cvBGRImg);
	}

	return;
}

void AstraData::getData_IR()
{
	// read frame  
	if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK)
	{
		cv::Mat tempImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
		cvRawImg16U = tempImg16U.clone();
		//cv::imshow("cvRawImg16U", cvRawImg16U);
		cvRawImg16U.convertTo(cvDepthImg8U, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
		//cv::imshow("depth", cvDepthImg8U);
	}
	if (oniIrStream.readFrame(&oniIrImg) == STATUS_OK)
	{
		// convert data into OpenCV type  
		cv::Mat IrImg(oniIrImg.getHeight(), oniIrImg.getWidth(), CV_16UC1, (void*)oniIrImg.getData());
		IrImg.convertTo(IrImg, CV_8U);
		cvIrImg = IrImg;
		//cv::imshow("image", IrImg);
		//waitKey(1);
	}

	return;
}