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
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinectProjectorToolkit.h"
#include "KinectGrabber.h"
#include "ofxModal.h"

#include "Utils.h"

class ofxModalThemeProjKinect : public ofxModalTheme {
public:
    ofxModalThemeProjKinect()
    {
        animation.speed = 0.1f;
        fonts.title = ofxSmartFont::add("ofxbraitsch/fonts/HelveticaNeueLTStd-Md.otf", 20, "modal-title");
        fonts.message = ofxSmartFont::add("ofxbraitsch/fonts/Roboto-Regular.ttf", 16, "modal-message");
    }
};

class KinectProjector {

public:
    void setup(ofVec2f sprojRes);
    void setupGradientField();
    
    void update();
    void updateCalibration();
    void updateFullAutoCalibration();
    
    void updateROIAutoCalibration();
    void updateROIFromColorImage();
    void updateROIFromDepthImage();
    void updateROIManualCalibration();
    void setMaxKinectGrabberROI();
    void setNewKinectROI();
    void updateKinectGrabberROI(ofRectangle ROI);
    
    void updateProjKinectAutoCalibration();
    void updateProjKinectManualCalibration();
    bool addPointPair();
    void updateMaxOffset();
    void updateBasePlane();
    
    void askToFlattenSand();
    
    void drawProjectorWindow();
    void drawMainWindow();
    void drawChessboard(int x, int y, int chessboardSize);
    void drawFlowField();
    void drawArrow(ofVec2f projectedPoint, ofVec2f v1);

    ofVec2f worldCoordToProjCoord(ofVec3f vin);
    ofVec2f kinectCoordToProjCoord(float x, float y);
    ofVec3f kinectCoordToWorldCoord(float x, float y);
    ofVec3f RawKinectCoordToWorldCoord(float x, float y);
    float elevationAtKinectCoord(float x, float y);
    float elevationToKinectDepth(float elevation, float x, float y);
    ofVec2f gradientAtKinectCoord(float x, float y);
    
    void updateMode();
    void updateNativeScale(float scaleMin, float scaleMax);
    
    void saveCalibrationAndSettings();
    bool loadSettings(string path);
    bool saveSettings(string path);
    
    void setupGui();
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onConfirmModalEvent(ofxModalEvent e);
    void onCalibModalEvent(ofxModalEvent e);
    
    void bind(){
        FilteredDepthImage.getTexture().bind();
    }
    void unbind(){
        FilteredDepthImage.getTexture().unbind();
    }
    ofRectangle getKinectROI(){
        return kinectROI;
    }
    ofVec2f getKinectRes(){
        return kinectRes;
    }
    ofTexture & getTexture(){
        return FilteredDepthImage.getTexture();
    }
    ofMatrix4x4 getTransposedKinectWorldMatrix(){
        return kinectWorldMatrix.getTransposedOf(kinectWorldMatrix);
    } // For shaders: OpenGL is row-major order and OF is column-major order
    ofMatrix4x4 getTransposedKinectProjMatrix(){
        return kinectProjMatrix.getTransposedOf(kinectProjMatrix);
    } // For shaders: OpenGL is row-major order and OF is column-major order
    ofVec4f getBasePlaneEq(){
        return basePlaneEq;
    }
    ofVec3f getBasePlaneNormal(){
        return basePlaneNormal;
    }
    ofVec3f getBasePlaneOffset(){
        return basePlaneOffset;
    }
    bool isCalibrating(){
        return calibrating;
    }
    bool isCalibrated(){
        return projKinectCalibrated;
    }
    bool isImageStabilized(){
        return imageStabilized;
    }
    bool isBasePlaneUpdated(){ // Can be called only once after update
        if (basePlaneUpdated){
            basePlaneUpdated = false;
            return true;
        } else{
            return false;
        }
    }
    bool isROIUpdated(){ // Can be called only once after update
        if (ROIUpdated){
            ROIUpdated = false;
            return true;
        } else{
            return false;
        }
    }
    bool isCalibrationUpdated(){ // Can be called only once after update
        if (projKinectCalibrationUpdated){
            projKinectCalibrationUpdated = false;
            return true;
        } else{
            return false;
        }
    }
    
private:
    enum Calibration_state
    {
        CALIBRATION_STATE_FULL_AUTO_CALIBRATION,
        CALIBRATION_STATE_ROI_AUTO_DETERMINATION,
        CALIBRATION_STATE_ROI_MANUAL_DETERMINATION,
        CALIBRATION_STATE_PROJ_KINECT_AUTO_CALIBRATION,
        CALIBRATION_STATE_PROJ_KINECT_MANUAL_CALIBRATION
    };
    enum Full_Calibration_state
    {
        FULL_CALIBRATION_STATE_ROI_DETERMINATION,
        FULL_CALIBRATION_STATE_AUTOCALIB,
        FULL_CALIBRATION_STATE_DONE
    };
    enum ROI_calibration_state
    {
        ROI_CALIBRATION_STATE_INIT,
        ROI_CALIBRATION_STATE_READY_TO_MOVE_UP,
        ROI_CALIBRATION_STATE_MOVE_UP,
        ROI_CALIBRATION_STATE_DONE
    };
    enum Auto_calibration_state
    {
        AUTOCALIB_STATE_INIT_FIRST_PLANE,
        AUTOCALIB_STATE_INIT_POINT,
        AUTOCALIB_STATE_NEXT_POINT,
        AUTOCALIB_STATE_COMPUTE,
        AUTOCALIB_STATE_DONE
    };

    // States variables
    bool ROIcalibrated;
    bool projKinectCalibrated;
    bool calibrating;
    bool ROIUpdated;
    bool projKinectCalibrationUpdated;
    bool basePlaneUpdated;
    bool imageStabilized;
    bool waitingForFlattenSand;
    Calibration_state calibrationState;
    ROI_calibration_state ROICalibState;
    Auto_calibration_state autoCalibState;
    Full_Calibration_state fullCalibState;

    //kinect interfaces
    KinectGrabber               kinectgrabber;
    ofxCvFloatImage FilteredDepthImage;
    ofVec2f* gradField;
    
    // Projector and kinect variables
    ofVec2f projRes;
    ofVec2f kinectRes;

    // FBos
    ofFbo fboProjWindow;
    ofFbo fboMainWindow;

    //Images and cv matrixes
    ofxCvColorImage             kinectColorImage;
    cv::Mat                     cvRgbImage;
    ofxCvFloatImage             Dptimg;
    
    //Gradient field variables
    int gradFieldcols, gradFieldrows;
    double gradFieldresolution;
    float arrowLength;
    
    // Calibration variables
    ofxKinectProjectorToolkit   kpt;
    vector<ofVec2f>             currentProjectorPoints;
    vector<cv::Point2f>         cvPoints;
    vector<ofVec3f>             pairsKinect;
    vector<ofVec2f>             pairsProjector;

    // ROI calibration variables
    ofxCvGrayscaleImage         thresholdedImage;
    ofxCvContourFinder          contourFinder;
    float threshold;
    ofPolyline large;
    ofRectangle                 kinectROI, kinectROIManualCalib;
    
    // Base plane
    ofVec3f basePlaneNormal, basePlaneNormalBack;
    ofVec3f basePlaneOffset, basePlaneOffsetBack;
    ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format
    
    // Conversion matrices
    ofMatrix4x4                 kinectProjMatrix;
    ofMatrix4x4                 kinectWorldMatrix;
    
    // Max offset for keeping kinect points
    float maxOffset;
    float maxOffsetSafeRange;
    float maxOffsetBack;
    
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

    // GUI Modal window
    shared_ptr<ofxModalConfirm>   confirmModal;
    shared_ptr<ofxModalAlert>   calibModal;
//    string                      resultMessage;
//    string                      modaltext;
//    ofColor                     resultMessageColor;
    // GUI Main interface
    ofxDatGui* gui;
};


#endif /* defined(__GreatSand__KinectProjector__) */


