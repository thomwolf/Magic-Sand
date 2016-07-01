#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinectProjectorToolkit.h"
#include "ofxDatGui.h"
#include "ofxModal.h"

#include "ColorMap.h"
#include "KinectGrabber.h"
#include "vehicle.h"
#include "Utils.h"

using namespace cv;
using namespace states;


class ofApp : public ofBaseApp{

public:

    void setup();
    void clearFbos();
    void setupMesh();
    void setRangesAndBasePlaneEquation();
    void setupGradientField();
    void setupVehicles();
    
    void update();
    
    void draw();
    void drawProjWindow(ofEventArgs& args);
    void drawChessboard(int x, int y, int chessboardSize);
    void drawTestingPoint(ofVec2f projectedPoint);
    
    void drawSandbox();
    void prepareContourLines();
    void prepareMotherPlateform();
    
    void drawFlowField();
    void drawArrow(ofVec2f projectedPoint, ofVec2f v1);
    
    void updateVehiclesFutureElevationAndGradient(vehicle& v);
    void drawVehicles();
    void drawMotherFish();
    void drawMotherRabbit();
    
    void drawTextGui();

    void addPointPair();
    void computeBasePlane();
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
    
    bool loadSettings(string path);
    bool saveSettings(string path);
    
    shared_ptr<ofAppBaseWindow> projWindow;
    
    ofxCvFloatImage FilteredDepthImage;
    ofVec2f* gradField;

private:
    //kinect interfaces and calibration
    KinectGrabber               kinectgrabber;
    ofxKinectProjectorToolkit   kpt;

    //Images and cv matrixes
    ofxCvColorImage             kinectColorImage;
    cv::Mat                     cvRgbImage;
    ofxCvFloatImage             Dptimg;
    
    ofxCvFloatImage vhcle;
    
    //Gradient field variables
    int gradFieldcols, gradFieldrows;
    double gradFieldresolution;
    float arrowLength;

    // Calibration variables
    vector<ofVec2f>             currentProjectorPoints;
    vector<cv::Point2f>         cvPoints;
    vector<ofVec3f>             pairsKinect;
    vector<ofVec2f>             pairsProjector;
    
    // Conversion matrices
    ofMatrix4x4                 kinectProjMatrix;
    ofMatrix4x4                 kinectWorldMatrix;

    // Gui variables
    string                      resultMessage;
    string                      modaltext;
    ofColor                     resultMessageColor;
    ofVec2f                     testPoint;
    bool saved;
    bool loaded;
    bool calibrated;
    bool firstImageReady;
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

    // ROI calibration variables
    ofxCvGrayscaleImage         thresholdedImage;
    ofxCvContourFinder          contourFinder;
    float threshold;
    ofPolyline large;
    ofRectangle                 kinectROI, kinectROIManualCalib;

    // Mesh
    ofMesh mesh;
    int meshwidth;          //Mesh size
    int meshheight;

    // Shaders
    ofShader elevationShader;
    ofShader heightMapShader;

    // FBos
    ofFbo fboProjWindow;
    ofFbo   contourLineFramebufferObject;

    // Base plane
    ofVec3f basePlaneNormal, basePlaneNormalBack;
    ofVec3f basePlaneOffset, basePlaneOffsetBack;
    ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format
    float maxOffset, maxOffsetSafeRange;
    
    // Autocalib points
    ofPoint* autoCalibPts; // Center of autocalib chess boards
    int currentCalibPts;
    bool cleared;
    int trials;
    bool upframe;
    
    // vehicules
    vector<Fish> fish;
    vector<Rabbit> rabbits;
    int fishNum, rabbitsNum;
    bool showMotherFish, showMotherRabbit;
    static const int MAX_STEPS = 10;
    
    ofPoint motherFish, motherRabbit; // Location of fish and rabbit mothers
    bool isMother;
    float motherPlatformSize;

    // Colormap, contourmap and heightmap variables
    ColorMap    heightMap;
	float heightMapScale,heightMapOffset; // Scale and offset values to convert from elevation to height color map texture coordinates
    float contourLineFboScale, contourLineFboOffset; // Scale and offset values to convert depth from contourline shader values to real values
	float FilteredDepthScale,FilteredDepthOffset; // Scale and offset values to convert depth from normalized shader values to real values

    // Contourlines
    float contourLineDistance, contourLineFactor;
    bool drawContourLines; // Flag if topographic contour lines are enabled
    
    float elevationMin, elevationMax;
    int   chessboardSize;
    int   chessboardX;
    int   chessboardY;
    int projResX;
    int projResY;
    int kinectResX;
    int kinectResY;
};
