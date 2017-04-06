#ifndef PROJECTOR_CAMERA
#define PROJECTOR_CAMERA

#include "Htime.h"
#define _USE_MATH_DEFINES
#include <Math.h>
//#include <android/log.h>
//#include "sendEvent.h"
#include <numeric>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//LOG DEFINITIONS
#define LOG_TAG_FLOW "ProjectorCamera/ProcessingFlow"
#define LOGF(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_FLOW, __VA_ARGS__))

#define LOG_TAG "ProjectorCamera/NearSurfaceInteraction"
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))

#define LOG_TAG_CALB "ProjectorCamera/NearSurfaceCalibration"
#define LOGC(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_CALB, __VA_ARGS__))

#define LOG_TAG_WARN "ProjectorCamera/NearSurfaceCalibration"
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_WARN, __VA_ARGS__))

#define LOGD printf
#define SCREEN_WIDTH_MM  744
#define SCREEN_HEIGHT_MM 434
#define CAMERA_HEIGHT_MM 800
#define SCREEN_WIDTH_PX  1280
#define SCREEN_HEIGHT_PX 720
#define SALVER_WIDTH_MM 190
#define DEPTH_SURFACE 650

//const int --can be modefied
const int cube_size = 200;
const int trajFrameNum = 8;
const int ntimes = 3;

//to be used
struct InAirGestureInfo{
	bool graspstate;
	int wavestate;
};

//in air hand state
//1.hand wave
enum HandWave{
	NOWAVE ,
	WAVERIGHT ,
	WAVELEFT
	//WAVEUP
	//WAVEDOWN
};
//2.hand grasp
enum HandGrasp{
	GRASPFAITTURE ,
	GRASPSUCCESS
};
//3.hand open
enum HandOpen{
	OPENFAITURE,
	OPENSUCCESS
};

using namespace cv;
using namespace std;

float PointDistence(Point2f&, Point2f&);

// colors
const Scalar COLOR_BLUE        = Scalar(240, 40, 0);
const Scalar COLOR_DARK_GREEN  = Scalar(0, 128, 0);
const Scalar COLOR_LIGHT_GREEN = Scalar(0, 255, 0);
const Scalar COLOR_YELLOW      = Scalar(0, 128, 200);
const Scalar COLOR_RED         = Scalar(0, 0, 255);

struct ConvexityDefect
{
	Point start;
	Point end;
	Point depth_point;
	float depth;
};

struct TouchInfo
{
	size_t lastId_mul,lastId_sig,firstId;
	bool   leftdown, multDown;
	Point2f lastPoint, lastPoint2;
	float distance,dx,dy;
	TouchInfo() :lastId_mul(0), lastId_sig(0),firstId(0),leftdown(0),
	             multDown(0), distance(0),dx(0),dy(0){};
};

struct HandContour
{
	vector<Point> approxCurve;
	Point finger, left, right;
	bool valid;
};

struct StereoProjection
{
	vector<Point2f> cameraVertex;
	vector<Point2f> projectorVertex;
	vector<Point3f> proVertex3D;
	vector<float>   vertexDepth;
	Point2f center;
	int lastId;
	bool valid;
	StereoProjection() :valid(false),lastId(0){};
};

struct TouchDetail
{
	Point2f tipPosition, bottomPosition, direction;
	float   angle,orien;
	float   tipDepth, bottomDepth;

	TouchDetail() :angle(0), tipDepth(0), bottomDepth(0),orien(0){};
	void getDirection();
	void getAngle();
	void getOrien();

	//lixing
	//palm center and depth
    float palmDepth;
    Point2f palmCenter;
    int frameNum;
    bool isReal;
};

struct VtouchPoints
{
	vector<TouchDetail> detailInfo;
	vector<Point2f>     cameraPts;
	vector<Point2f>     projecPts;
	float dx, dy;
	void clear()
	{
	    detailInfo.clear();
	    cameraPts.clear();
	    projecPts.clear();
	}
};

struct ObjectInfo{
    Mat object;
    float maxdepth,cArea;
    vector<float> angles;
    vector<Point> contour;
};

struct FingerParas
{
	FingerParas() :minTipAngle(70), minBgFgBias(8), unchangeRadius(150){};
	float minTipAngle;
	float minBgFgBias;
	float unchangeRadius;
};

class ProjectorCamera
{
public:
	ProjectorCamera();
	~ProjectorCamera();

	void init();
	void processing(Mat& colorSrc,Mat& depthSrc);
	bool histcontinue(Mat&);
	void calibration();
	void calibrationFixed();
	void calibrationPerspective();
	void getBgDepth();
	void getFgDepth();
	void getContour();
	void findFinger();
	void findOnDeskObject();
	void findInAirObject();
	void findInAirGesture();
	void refineRect(Rect& rec, Mat image);
	void refineFingerPosition(int);
	void transAxisCameraToPro();
	void fillDepthImageHole(Mat &);
	float contourAverageDepth(vector<Point>&);
	float caclAngle(Point& left, Point& center, Point& right);
	//float calPointDistance(Point& pointA, Point& pointB);
	float calPointDistance(Point2f pointA, Point2f pointB);
	float calPointDistance(Point3f pointA, Point3f pointB);
	string  intTostring(int);
    Point2f fingerDirection(Point2f, Point2f, Point2f);
    void convetFloatToChar(Mat& src,Mat& dst);
	//zhangbo
	bool isHandGraspOrOpen(double& handRatio, int& approxCurveSize, int& hullsize, int& littleangle,int& distance_x,Point& centerpoint);
    int handWaveState(vector<Point>& pointsInLastFrames);
	Point CalculateSumofPoints(Point pointA, Point pointB);
    void findConvexityDefects(vector<Point>& contour, vector<int>& hull, vector<ConvexityDefect>& convexDefects);
    void inAirGestureManage(bool& openstate, int& handwave);
    void pointCamToPro(vector<Point2f>&,vector<float>&,vector<Point3f>&);
    void pointCamToPro(vector<Point2f>&,vector<float>&,vector<Point2f>&);
    void refineVerticals(vector<Point3f>&);
    void clockwiseContour(vector<Point2f>&);
    void clockwiseContour(vector<Point>&);

	Mat colorImg, depthImg, irimage, depthImg_old,diffeImg;
	Mat averaImg, foreground, foreground_store, foreground_copy, averaIR;	
	Mat hist,h;
	vector<Mat> bgdepths;
	Rect screenRoi,filterRoi;

	bool calibrated, initBg;
	size_t  frameID, initFrames;

	vector<Point2f> touchPoints,rightPoints;

	float nearSurFaceThresh,nearCameraThresh;
	double maxdepth, bfdis;
	int histMinPixs;

	//calibration
	int gridstart_x, gridstart_y,gridwidth;
	int offset_x,offset_y,offset_init_x,offset_init_y;
		
	TouchInfo touchInfo;
	VtouchPoints vtouchPoints;

	StereoProjection stereoProjectDesk, stereoProjectHover;

	int fd,appState;

	vector<ObjectInfo> objects;
	CVTIME getBg;

    //zhangbo
    //in air gesture variables
    int ncount_totalimg, ncount_grasp;
    int ncount_totalimg_open, ncount_open;
    vector<Point>pointsInLastFrames;
    //InAirGestureInfo inairgesture;
    vector<bool> handgraspstates_temp;//hand grasp
    vector<int> handwavestates;//hand wave
    bool graspstate_temp;

    int lastframe;
    int lastframe_grasp;
    //int lastframe_wave;

    bool m_graspstate;
    bool m_openstate;
    int m_wavestate;

	//output info-->for android
	bool graspstate_finnal;
	bool openstate_finnal;
	int wavestate_finnal;


    //baoyou
    Mat transform;
	//void ImageToPro(const vector<Point2f>& vpoints, vector<Point3d>& corners);
	//void distance(const vector<Point3d>& p,vector<float>& len);
	//void distance(const vector<Point3d>& p,vector<float>& len, vector<float>& angle);
	//void refineCornerDetection(const vector<Point2f>& vpoints, vector<Point2f>& refined);
	//int dsize;  //the size of neighbor area
    //vector<Point2f> saved_point;  //the cordinate previous 4 corners.

    //procamera calibration
    string cablibParaFilePath;
    double c_fx,c_fy,c_cx,c_cy;
    double p_fx,p_fy,p_cx,p_cy;
    Mat camToProR, camToProT;
    Mat camera_KK, camera_dis;
    Mat project_KK, project_dis;

    //lixing
    double minVal, maxVal;
    Point minLoc, maxLoc;
    Mat bincp;
};




#endif