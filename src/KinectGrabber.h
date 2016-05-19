#pragma once
#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

#include "FrameFilter.h"
#include "Utils.h"

//#include <Geometry/HVector.h>
//#include <Geometry/Plane.h>
//#include <Geometry/Matrix.h>
//#include <Geometry/ProjectiveTransformation.h>

using namespace states;

class KinectGrabber: public ofThread {
public:
	/* Embedded classes: */
//	typedef Geometry::Plane<double,3> Plane;
//	typedef Geometry::ProjectiveTransformation<double,3> PTransform;

	/* Elements: */
	KinectGrabber();
	~KinectGrabber();
    void setup(General_state, Calibration_state);
    void setupClip(float nearclip, float farclip);
    void setupFramefilter(int gradFieldresolution, float snearclip, float sfarclip,const ofVec3f basePlaneNormal, double MinElevation,double MaxElevation, ofRectangle ROI);
    void setupCalibration(int projectorWidth, int projectorHeight, float schessboardSize, float schessboardColor, float sStabilityTimeInMs, float smaxReprojError);
    void setMode(General_state sgeneralState, Calibration_state scalibrationState);
    void setKinectROI(ofRectangle skinectROI);
    //void update();
//    ofPixels convertProjSpace(ofPixels sinputframe);
    ofVec2f getKinectSize();
    ofMatrix4x4 getWorldMatrix();
	bool isFrameNew();
    int storedframes;//, storedcoloredframes;
	ofPixels & getPixels();
	ofTexture & getTexture();
//    PTransform getProjMatrix(void); // Get unprojection matrix of the kinect
//    ofVec3f getProjVector(void); // Kinect unprojection factors: (shift x, shift y, scale factor)
    
	ofThreadChannel<ofFloatPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<ofVec2f*> gradient;
	ofThreadChannel<General_state> generalStateChannel;
	ofThreadChannel<Calibration_state> calibrationStateChannel;

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
    
    // kinect & the wrapper
    float                   nearclip, farclip;
//    float depthNorm;
    int kinectWidth, kinectHeight;//, projWidth, projHeight;
    ofxCvColorImage         kinectColorImage;
//    ofxCvGrayscaleImage		kinectGreyscaledImage;
    ofShortPixels     kinectDepthImage;
    //   ofImage                 kinectColoredDepth;
    float maxReprojError;
    // calibration
    // output
    General_state generalState;
    Calibration_state calibrationState;
};
