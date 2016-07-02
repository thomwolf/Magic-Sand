#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "ofxModal.h"

#include "KinectProjector.h"
#include "SandSurfaceRenderer.h"
#include "vehicle.h"
#include "Utils.h"

using namespace cv;
using namespace states;


class ofApp : public ofBaseApp{

public:

    void setup();
    void clearFbos();
    void setupVehicles();
    
    void update();
    
    void draw();
    void drawProjWindow(ofEventArgs& args);
    
    void updateVehiclesFutureElevationAndGradient(vehicle& v);
    void drawVehicles();
    void drawMotherFish();
    void drawMotherRabbit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    void setupGui();
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onModalEvent(ofxModalEvent e);
    
    shared_ptr<ofAppBaseWindow> projWindow;
    
private:
    ofxCvFloatImage vhcle;
    // Projector variables
    int projResX;
    int projResY;
    
    // Gui variables
    string                      resultMessage;
    string                      modaltext;
    ofColor                     resultMessageColor;
    bool waitingToInitialiseVehicles;
    // GUI
    ofxDatGui* gui;
    // Modal window
    shared_ptr<ofxModalAlert> modal;
    uint tIndex;
    vector<ofxDatGuiTheme*> themes;

    // States variables
    General_state generalState, previousGeneralState;
    Calibration_state calibrationState, previousCalibrationState;
    ROI_calibration_state ROICalibrationState;
    Autocalib_calibration_state autoCalibState;
    Initialisation_state initialisationState;

    // FBos
    ofFbo fboProjWindow;

    // vehicules
    vector<Fish> fish;
    vector<Rabbit> rabbits;
    int fishNum, rabbitsNum;
    bool showMotherFish, showMotherRabbit;
    static const int MAX_STEPS = 10;
    
    ofPoint motherFish, motherRabbit; // Location of fish and rabbit mothers
    bool isMother;
    float motherPlatformSize;

};
