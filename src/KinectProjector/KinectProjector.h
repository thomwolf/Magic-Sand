/***********************************************************************
KinectProjector - KinectProjector takes care of the spatial conversion
between the various coordinate systems, control the kinectgrabber and
perform the calibration of the kinect and projector.
Copyright (c) 2016 Thomas Wolf

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef __GreatSand__KinectProjector__
#define __GreatSand__KinectProjector__

#include <iostream>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "KinectGrabber.h"
#include "ofxModal.h"

#include "KinectProjectorCalibration.h"
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
    KinectProjector(std::shared_ptr<ofAppBaseWindow> const& p);
    
    // Running loop functions
    void setup(bool sdisplayGui);
    void update();
    void updateNativeScale(float scaleMin, float scaleMax);
    void drawProjectorWindow();
    void drawMainWindow(float x, float y, float width, float height);
    void drawGradField();

    // Coordinate conversion functions
    ofVec2f worldCoordToProjCoord(ofVec3f vin);
	ofVec3f projCoordAndWorldZToWorldCoord(float projX, float projY, float worldZ);
	ofVec2f kinectCoordToProjCoord(float x, float y);
    ofVec3f kinectCoordToWorldCoord(float x, float y);
	ofVec2f worldCoordTokinectCoord(ofVec3f wc);
	ofVec3f RawKinectCoordToWorldCoord(float x, float y);
    float elevationAtKinectCoord(float x, float y);
    float elevationToKinectDepth(float elevation, float x, float y);
    ofVec2f gradientAtKinectCoord(float x, float y);
    
    // Setup & calibration functions
    void startFullCalibration();
    void startAutomaticROIDetection();
    void startAutomaticKinectProjectorCalibration();
    void setGradFieldResolution(int gradFieldResolution);
    void setSpatialFiltering(bool sspatialFiltering);
    void setFollowBigChanges(bool sfollowBigChanges);
    
    // Gui and event functions
    void setupGui();
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onConfirmModalEvent(ofxModalEvent e);
    void onCalibModalEvent(ofxModalEvent e);
    
    // Functions for shaders
    void bind(){
        FilteredDepthImage.getTexture().bind();
    }
    void unbind(){
        FilteredDepthImage.getTexture().unbind();
    }
    ofMatrix4x4 getTransposedKinectWorldMatrix(){
        return kinectWorldMatrix.getTransposedOf(kinectWorldMatrix);
    } // For shaders: OpenGL is row-major order and OF is column-major order
    ofMatrix4x4 getTransposedKinectProjMatrix(){
        return kinectProjMatrix.getTransposedOf(kinectProjMatrix);
    }
    
    // Getter and setter
    ofTexture & getTexture(){
        return FilteredDepthImage.getTexture();
    }
    ofRectangle getKinectROI(){
        return kinectROI;
    }
    ofVec2f getKinectRes(){
        return kinectRes;
    }
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
    bool isBasePlaneUpdated(){ // To be called after update()
        return basePlaneUpdated;
    }
    bool isROIUpdated(){ // To be called after update()
        return ROIUpdated;
    }
    bool isCalibrationUpdated(){ // To be called after update()
        return projKinectCalibrationUpdated;
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

    // Private methods
    void exit(ofEventArgs& e);
    void setupGradientField();
    
    void updateCalibration();
    void updateFullAutoCalibration();
    void updateROIAutoCalibration();
    void updateROIFromColorImage();
    void updateROIFromDepthImage();
    void updateROIManualCalibration();
    void updateROIFromCalibration();
    void setMaxKinectGrabberROI();
    void setNewKinectROI();
    void updateKinectGrabberROI(ofRectangle ROI);
    
    void updateProjKinectAutoCalibration();
    void updateProjKinectManualCalibration();
    bool addPointPair();
    void updateMaxOffset();
    void updateBasePlane();
    void askToFlattenSand();

    void drawChessboard(int x, int y, int chessboardSize);
    void drawArrow(ofVec2f projectedPoint, ofVec2f v1);

    void saveCalibrationAndSettings();
    bool loadSettings();
    bool saveSettings();
    
    // States variables
    bool secondScreenFound;
	bool kinectOpened;
    bool ROIcalibrated;
    bool projKinectCalibrated;
    bool calibrating;
    bool ROIUpdated;
    bool projKinectCalibrationUpdated;
    bool basePlaneUpdated;
    bool imageStabilized;
    bool waitingForFlattenSand;
    bool drawKinectView;
    Calibration_state calibrationState;
    ROI_calibration_state ROICalibState;
    Auto_calibration_state autoCalibState;
    Full_Calibration_state fullCalibState;

    // Projector window
    std::shared_ptr<ofAppBaseWindow> projWindow;
    
    //kinect grabber
    KinectGrabber               kinectgrabber;
    bool                        spatialFiltering;
    bool                        followBigChanges;
    int                         numAveragingSlots;

    //kinect buffer
    ofxCvFloatImage             FilteredDepthImage;
    ofxCvColorImage             kinectColorImage;
    ofVec2f*                    gradField;
    
    // Projector and kinect variables
    ofVec2f projRes;
    ofVec2f kinectRes;

    // FBos
    ofFbo fboProjWindow;
    ofFbo fboMainWindow;

    //Images and cv matrixes
    cv::Mat                     cvRgbImage;
    ofxCvFloatImage             Dptimg;
    
    //Gradient field variables
    int gradFieldcols, gradFieldrows;
    int gradFieldResolution;
    float arrowLength;
    int fishInd;
    
    // Calibration variables
    ofxKinectProjectorToolkit*  kpt;
    vector<ofVec2f>             currentProjectorPoints;
    vector<cv::Point2f>         cvPoints;
    vector<ofVec3f>             pairsKinect;
    vector<ofVec2f>             pairsProjector;

    // ROI calibration variables
    ofxCvGrayscaleImage         thresholdedImage;
    ofxCvContourFinder          contourFinder;
    float                       threshold;
    ofPolyline                  large;
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

    // GUI Modal window & interface
	bool displayGui;
    shared_ptr<ofxModalConfirm>   confirmModal;
    shared_ptr<ofxModalAlert>   calibModal;
    shared_ptr<ofxModalThemeProjKinect>   modalTheme;
    ofxDatGui* gui;
};


#endif /* defined(__GreatSand__KinectProjector__) */


