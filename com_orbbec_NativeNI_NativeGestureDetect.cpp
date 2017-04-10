/********************************************************************
Projection time:	2016/10/31   15:53
File Name: 	com_orbbec_NativeNI_NativeGestureDetect.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2016, NetEase Inc. All rights reserved.

note:       Projector-camera Android ֲJNI File
*********************************************************************/

#include "jni.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/operations.hpp"
#include "projectorCamera.h"
#include <string>
#include <vector>
#include <stdlib.h>
#include <exception>  
//#include <android/log.h>
#include "json.h"
#include <algorithm>

#ifdef __cplusplus
extern "C" {
#endif


/*!
@function
@abstract              Jstring->string
@discussion
@param     jstring     输入J字符串
@result    string      输出C字符串
*/
std::string ConvertJstr2Str(JNIEnv *env, jstring &jstrIn)
{
	const char *strTmp = env->GetStringUTFChars(jstrIn, NULL);
	std::string str = strTmp;
	env->ReleaseStringUTFChars(jstrIn, strTmp);
	return str;
}

/*!
@function
@abstract              string->Jstring
@discussion
@param     string      输入C字符串
@result    jstring     输出J字符串
*/
jstring ConvertStr2Jstr(JNIEnv *env, string &jstrIn)
{
	return env->NewStringUTF(jstrIn.c_str());
}

/*!
@function
@abstract
@discussion
@param     proCameraPtr
@param     proCameraPtr
@result    jstring
*/
string AndroidResponds(ProjectorCamera* proCameraPtr)
{
	jstring out;
	Json::Value jroot,jlist,jresult;
	int eventInterval = 2;

	//on the desk stereoProjectDesk updata
    if((proCameraPtr->appState == 1 || proCameraPtr->appState == 2) && proCameraPtr->stereoProjectDesk.valid)
    {
        Json::Value vinfo;
        for (int ind = 1; ind < 4;ind++)
        {
            //以桌面的右上为unity坐标原点，右下为投影仪的坐标原点
        	Json::Value jinfo;
        	jinfo["x"] = proCameraPtr->stereoProjectDesk.proVertex3D[ind].x / 1000;
        	jinfo["y"] = proCameraPtr->stereoProjectDesk.proVertex3D[ind].y / 1000;
        	jinfo["z"] = proCameraPtr->stereoProjectDesk.proVertex3D[ind].z / 1000;

        	vinfo.append(jinfo);
        }

        jresult["state"] = 1;
        jresult["points"] = vinfo;
        jlist.append(jresult);
        //jroot["list"] = jlist;
        proCameraPtr->stereoProjectDesk.valid = false;
    }

	//in the air stereoProject updata
    if((proCameraPtr->appState == 1 || proCameraPtr->appState == 2) && proCameraPtr->stereoProjectHover.valid)
    {
        Json::Value vinfo;
        for (int ind = 1; ind < 4;ind++)
        {
            //以桌面的右上为unity坐标原点，右下为投影仪的坐标原点
        	Json::Value jinfo;
            jinfo["x"] = proCameraPtr->stereoProjectHover.proVertex3D[ind].x / 1000 ;
        	jinfo["y"] = proCameraPtr->stereoProjectHover.proVertex3D[ind].y / 1000 ;
        	jinfo["z"] = proCameraPtr->stereoProjectHover.proVertex3D[ind].z / 1000;
        	vinfo.append(jinfo);
        }

        jresult["state"] = 2;
        jresult["points"] = vinfo;
        jlist.append(jresult);

        proCameraPtr->stereoProjectHover.valid = false;
    }
    jroot["list"] = jlist;

	//in the air gesture updata
    if(proCameraPtr->graspstate_finnal)
    {
        EVENT::chosen(proCameraPtr->fd);
        proCameraPtr->graspstate_finnal = false;
    }
    if(proCameraPtr->wavestate_finnal == 1)
    {
        EVENT::sweptright(proCameraPtr->fd);
        proCameraPtr->wavestate_finnal = 0;          
    }
    if(proCameraPtr->wavestate_finnal == 2)
    {
        EVENT::sweptleft(proCameraPtr->fd);
        proCameraPtr->wavestate_finnal = 0;       
    }
    if(proCameraPtr->openstate_finnal)
    {
        EVENT::back(proCameraPtr->fd);
        proCameraPtr->openstate_finnal = false;
    }

	//finger touch updata
    proCameraPtr->fingerTouchRole.update();
    //LOGD("touchPoint current state %s",proCameraPtr->fingerTouchRole.currentState->getDescription().c_str());

	//jout = ConvertStr2Jstr(env, jresult.toStyledString());
    //LOGD("projector string %s",jroot.toStyledString().c_str());
	return jroot.toStyledString();
}


/*!
@function
@abstract     initialization class ProjectorCamera
@discussion
@param
@result    jlong
*/
 jlong  Java_com_orbbec_NativeNI_NativeGestureDetect_nativeInit
(JNIEnv *jenv, jobject,jstring jsin,jstring filePath)
{
	LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_nativeInit enter");
	jlong result = 0;

	try
	{
		LOGF("Creating ProjectorCamera...");
		result = (jlong) new(ProjectorCamera);

		string strPara = ConvertJstr2Str(jenv,jsin);
		Json::Reader reader;
        Json::Value root;
        LOGF("input parameters json: %s", strPara.c_str());

        string strcalib= ConvertJstr2Str(jenv,filePath);
        LOGF("input parameters json: %s", strcalib.c_str());
        ((ProjectorCamera*)result)->cablibParaFilePath = ConvertJstr2Str(jenv,filePath);
        
	    // reader read Json string to root
        if (reader.parse(strPara, root))
        {
             LOGF("reset parameters form json: %s", "start write");
             ((ProjectorCamera*)result)->screenRoi.x = root["Rect_X"].asInt();
             ((ProjectorCamera*)result)->screenRoi.y = root["Rect_Y"].asInt();
             ((ProjectorCamera*)result)->screenRoi.width  = root["Rect_W"].asInt();
             ((ProjectorCamera*)result)->screenRoi.height = root["Rect_H"].asInt();
             ((ProjectorCamera*)result)->filterRoi.x = root["Filt_X"].asInt();
             ((ProjectorCamera*)result)->filterRoi.y = root["Filt_Y"].asInt();
             ((ProjectorCamera*)result)->filterRoi.width  = root["Filt_W"].asInt();
             ((ProjectorCamera*)result)->filterRoi.height = root["Filt_H"].asInt();
             ((ProjectorCamera*)result)->gridstart_x = root["Grid_X"].asInt();
             ((ProjectorCamera*)result)->gridstart_y = root["Grid_Y"].asInt();
             ((ProjectorCamera*)result)->offset_init_x = root["Offset_X"].asInt();
             ((ProjectorCamera*)result)->offset_init_y = root["Offset_Y"].asInt();
        }
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeCreateObject caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeCreateObject caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of NativeGestureDetect_nativeInit()");
		return 0;
	}

	return result;
}


/*!
@function
@abstract      destroy class ProjectorCamera
@discussion
@param     jlong       ProjectorCamera pointer
@result    jboolean
*/
 jboolean  Java_com_orbbec_NativeNI_NativeGestureDetect_nativeDestroy
(JNIEnv *jenv, jobject, jlong proCameraPtr)
{
	LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_nativeDestroy");
	try
	{
		if (proCameraPtr != 0)
		{			
			delete (ProjectorCamera*)proCameraPtr;
		}
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeestroyObject caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeDestroyObject caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of NativeGestureDetect_nativeDestroy()");
	}

	return jboolean(1);
}


/*!
@function
@abstract        projector_camera calibration
@discussion
@param     jlong       ProjectorCamera pointer
@result    jboolean
*/
 jboolean  Java_com_orbbec_NativeNI_NativeGestureDetect_calibration
(JNIEnv *jenv, jobject, jlong proCameraPtr, jlong rgbPt, jint chressGridSize)
{
	LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_calibration");
	jboolean calibrated(false);

	try
	{
		//get color image
		//Mat rgbImg(*(Mat *)rgbPt);

		((ProjectorCamera*)proCameraPtr)->gridwidth = chressGridSize;
		//((ProjectorCamera*)proCameraPtr)->colorImg  = rgbImg;
		//((ProjectorCamera*)proCameraPtr)->calibration();
        ((ProjectorCamera*)proCameraPtr)->calibrationFixed();
		calibrated = ((ProjectorCamera*)proCameraPtr)->calibrated;
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeStart caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeStart caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of NativeGestureDetect_calibration()");
	}

     return calibrated;
}

/*!
@function
@abstract              projector_camera calibration
@discussion
@param     jlong       ProjectorCamera pointer
@result    jboolean
*/
 jboolean  Java_com_orbbec_NativeNI_NativeGestureDetect_calibrationFixed
(JNIEnv *jenv, jobject, jlong proCameraPtr)
{
	LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_calibration_fixed");
	jboolean calibrated(false);

	try
	{
		((ProjectorCamera*)proCameraPtr)->calibrationFixed();
		calibrated = ((ProjectorCamera*)proCameraPtr)->calibrated;
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeStart caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeStart caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of NativeGestureDetect_calibration_fixed()");
	}

     return calibrated;
}

/*!
@function
@abstract              projector_camera system reset
@discussion
@param     jlong       ProjectorCamera pointer
@result    jboolean
*/
 jboolean  Java_com_orbbec_NativeNI_NativeGestureDetect_reset
(JNIEnv *jenv, jobject, jlong proCameraPtr)
{
	LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_calibration_fixed");
	jboolean calibrated(false);

	try
	{
		//((ProjectorCamera*)proCameraPtr)->reset();
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeStart caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeStart caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of NativeGestureDetect_calibration_fixed()");
	}

     return calibrated;
}

/*!
@function
@abstract               projector_camera system detection
@discussion
@param     proCameraPtr ProjectorCamera pointer
@param     rgbPt        color image pointer
@param     depthPt      depth image pointer
@result    jstring
*/
 jstring  Java_com_orbbec_NativeNI_NativeGestureDetect_getNativeGestureResult
(JNIEnv *jenv, jobject, jlong proCameraPtr, jlong iRPt, jlong depthPt, jint offset_x, jint offset_y,jint appState)
{
	//LOGF("Java_com_orbbec_NativeNI_NativeGestureDetect_getNativeGestureResult");
	jstring jstrout;

	try
	{
		//get rgb and depth image
		Mat iRImg(*(Mat *)iRPt);
		Mat depImg(*(Mat *)depthPt);

		((ProjectorCamera*)proCameraPtr)->offset_x = offset_x;
		((ProjectorCamera*)proCameraPtr)->offset_y = offset_y;
		((ProjectorCamera*)proCameraPtr)->appState = appState;
		//LOGF("nativeStart caught offset_x: %f" , offset_x);
		//LOGF("nativeStart caught offset_y: %f" , offset_y);
		 
		((ProjectorCamera*)proCameraPtr)->processing(iRImg,depImg);

         string out = AndroidResponds((ProjectorCamera*)proCameraPtr);

         jstrout = ConvertStr2Jstr(jenv, out);

         //LOGD("stereo projection json out: %s", out.c_str());
	}
	catch (cv::Exception& e)
	{
		LOGF("nativeStart caught cv::Exception: %s", e.what());
		jclass je = jenv->FindClass("org/opencv/core/CvException");
		if (!je)
			je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, e.what());
	}
	catch (...)
	{
		LOGF("nativeStart caught unknown exception");
		jclass je = jenv->FindClass("java/lang/Exception");
		jenv->ThrowNew(je, "Unknown exception in JNI code of DetectionBasedTracker.nativeStart()");
	}

	
	return jstrout;
}
#ifdef __cplusplus
}
#endif