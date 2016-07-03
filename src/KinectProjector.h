//
//  KinectProjector.h
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#ifndef __GreatSand__KinectProjector__
#define __GreatSand__KinectProjector__

#include <iostream>
#include "ofMain.h"
#include "KinectGrabber.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinectProjectorToolkit.h"
#include "ofxModal.h"

#include "Utils.h"

#endif /* defined(__GreatSand__KinectProjector__) */

using namespace states;

class KinectProjector {

public:
    void setup(int sprojResX, int sprojResY);
    void setupGradientField();
    void updateNativeScale(float scaleMin, float scaleMax);
    void clearFbos();

    void update();
//    const std::vector<double>& x() const { return x_; }
   
    void drawChessboard(int x, int y, int chessboardSize);
    void drawTestingPoint(ofVec2f projectedPoint);
    void drawFlowField();
    void drawArrow(ofVec2f projectedPoint, ofVec2f v1);

    void addPointPair();
    void findMaxOffset();
    ofVec2f computeTransform(ofVec4f vin);
    ofVec4f getWorldCoord(float x, float y);
    
    void updateROIAutoSetup();
    void updateROIFromColorImage();
    void updateROIFromDepthImage();
    void updateROIManualSetup();
    void updateKinectGrabberROI();
    void autoCalib();
    void updateMode();
    
    bool loadSettings(string path);
    bool saveSettings(string path);
    void onModalEvent(ofxModalEvent e);
    
    ofRectangle getKinectROI(){
        return kinectROI;
    }
    ofVec2f getKinectRes(){
        return ofVec2f(kinectResX, kinectResY);
    }
    ofTexture & getTexture(){
        return FilteredDepthImage.getTexture();
    }
    ofMatrix4x4 getTransposedKinectWorldMatrix(){
        return kinectWorldMatrix.getTransposedOf(kinectWorldMatrix);
    }
    ofMatrix4x4 getTransposedKinectProjMatrix(){
        return kinectProjMatrix.getTransposedOf(kinectProjMatrix);
    }
    
private:
    // States variables
    General_state generalState, previousGeneralState;
    Calibration_state calibrationState, previousCalibrationState;
    ROI_calibration_state ROICalibrationState;
    Autocalib_calibration_state autoCalibState;
    Initialisation_state initialisationState;

    // GUI Modal window
    shared_ptr<ofxModalAlert>   modal;
    string                      resultMessage;
    string                      modaltext;
    ofColor                     resultMessageColor;

    bool saved;
    bool loaded;
    bool calibrated;
    bool firstImageReady;

    //kinect interfaces and calibration
    KinectGrabber               kinectgrabber;
    ofxKinectProjectorToolkit   kpt;
    ofxCvFloatImage FilteredDepthImage;
    ofVec2f* gradField;
    
    // Projector and kinect variables
    int projResX;
    int projResY;
    int kinectResX;
    int kinectResY;

    //Images and cv matrixes
    ofxCvColorImage             kinectColorImage;
    cv::Mat                     cvRgbImage;
    ofxCvFloatImage             Dptimg;
    
    //Gradient field variables
    int gradFieldcols, gradFieldrows;
    double gradFieldresolution;
    float arrowLength;
    
    // Calibration variables
    vector<ofVec2f>             currentProjectorPoints;
    vector<cv::Point2f>         cvPoints;
    vector<ofVec3f>             pairsKinect;
    vector<ofVec2f>             pairsProjector;
    ofVec2f                     testPoint;

    // ROI calibration variables
    ofxCvGrayscaleImage         thresholdedImage;
    ofxCvContourFinder          contourFinder;
    float threshold;
    ofPolyline large;
    ofRectangle                 kinectROI, kinectROIManualCalib;
    
    // Conversion matrices
    ofMatrix4x4                 kinectProjMatrix;
    ofMatrix4x4                 kinectWorldMatrix;
    
    // Max offset for keeping kinect points
    float maxOffset, maxOffsetSafeRange;
    
    // Autocalib points
    ofPoint* autoCalibPts; // Center of autocalib chess boards
    int currentCalibPts;
    bool cleared;
    int trials;
    bool upframe;
    
    // Chessboard variables
    int   chessboardSize;
    int   chessboardX;
    int   chessboardY;
};
