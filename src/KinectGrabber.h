#pragma once
#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

#include "FrameFilter.h"

#include <Geometry/HVector.h>
#include <Geometry/Plane.h>
#include <Geometry/Matrix.h>
#include <Geometry/ProjectiveTransformation.h>

class KinectGrabber: public ofThread {
public:
	/* Embedded classes: */
	typedef Geometry::Plane<double,3> Plane;
	typedef Geometry::ProjectiveTransformation<double,3> PTransform;

	/* Elements: */
	KinectGrabber();
	~KinectGrabber();
    void setup();
    void setupClip(float nearclip, float farclip);
    void setupFramefilter(int sNumAveragingSlots, int gradFieldresolution, float snearclip, float sfarclip,const FrameFilter::PTransform& depthProjection,const FrameFilter::Plane& basePlane, double elevationMin, double elevationMax);
    void setupCalibration(int projectorWidth, int projectorHeight, float schessboardSize, float schessboardColor, float sStabilityTimeInMs, float smaxReprojError);
    void setCalibrationmode();
    void setTestmode();
    void setKinectROI(ofRectangle skinectROI);
    //void update();
//    ofPixels convertProjSpace(ofPixels sinputframe);
	bool isFrameNew();
    int storedframes;//, storedcoloredframes;
	ofPixels & getPixels();
	ofTexture & getTexture();
    PTransform getProjMatrix(void); // Get unprojection matrix of the kinect
    ofVec3f getProjVector(void); // Kinect unprojection factors: (shift x, shift y, scale factor)
    
	ofThreadChannel<ofFloatPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<ofVec2f*> gradient;
	ofThreadChannel<float> nearclipchannel;
	ofThreadChannel<float> farclipchannel;

    ofxKinect               kinect;
//    float                       lowThresh;
//    float                       highThresh;
    float                       chessboardThreshold;
    // Framefilter
    FrameFilter                 framefilter;

private:
	void threadedFunction();
//	ofThreadChannel<ofPixels> toAnalyze;
	ofPixels pixels;
	ofTexture texture;
	bool newFrame;
	bool enableCalibration, enableTestmode;
    
    // kinect & the wrapper
    float                   nearclip, farclip;
    int kinectWidth, kinectHeight;//, projWidth, projHeight;
    ofxCvColorImage         kinectColorImage;
//    ofxCvGrayscaleImage		kinectGreyscaledImage;
    ofShortPixels     kinectDepthImage;
    //   ofImage                 kinectColoredDepth;
    float maxReprojError;
    ofRectangle                 kinectROI;
    // calibration
    // output
};
