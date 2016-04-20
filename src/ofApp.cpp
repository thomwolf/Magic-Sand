#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

#define GENERAL_STATE_CALIBRATION = 0,
#define GENERAL_STATE_SANDBOX = 1
#define CALIBRATION_STATE_ROI_DETERMINATION = 0,
#define CALIBRATION_STATE_PROJ_KINECT_CALIBRATION = 1

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
	
	// settings and defaults
	generalState = GENERAL_STATE_CALIBRATION;
	calibrationState  = CALIBRATION_STATE_ROI_DETERMINATION;
	kinectROI = ofRectangle(0, 0, 640, 480);
	
	//Mesh calib
	meshwidth = 2; // 640 should be dividable by meshwidth-1
	meshheight = 2; // idem with 480
	
	// kinect depth clipping
	nearclip = 750;
	farclip = 950;
	double elevationMin=-1000.0;
	double elevationMax=1000.0;
	int numAveragingSlots=30;
	unsigned int minNumSamples=10;
	unsigned int maxVariance=2;
	float hysteresis=0.1f;
	bool spatialFilter=false;
	gradFieldresolution = 20;
    
	// Load colormap
    heightMap.load("HeightColorMap.yml");
	
	/* Limit the valid elevation range to the extent of the height color map: */
	//if(elevationMin<heightMap.getScalarRangeMin())
	elevationMin=heightMap.getScalarRangeMin();
	//if(elevationMax>heightMap.getScalarRangeMax())
	elevationMax=heightMap.getScalarRangeMax();
	
    // kinectgrabber: setup
	kinectgrabber.setup();
	//	kinectgrabber.setupClip(nearclip, farclip);
	
    // calibration config
	chessboardSize = 300;
	chessboardX = 5;
    chessboardY = 4;
    fboChessboard.allocate(PROJECTOR_RESOLUTION_X, PROJECTOR_RESOLUTION_Y, GL_RGBA);
	
//    horizontalMirror = false;//true;
//	verticalMirror = false;//true;
	
	basePlane = Geometry::Plane<double, 3>(Geometry::Vector<double, 3>(0,0,1),Geometry::Point<double, 3>(0,0,-870));
	
	kinectgrabber.setupFramefilter(numAveragingSlots, gradFieldresolution,nearclip, farclip, depthProjection, basePlane, elevationMin, elevationMax);
	kinectgrabber.setKinectROI(kinectROI);
//    setupView();
	//kinectProjectorOutput.load("kinectProjector.yml");
	
    // prepare shaders and fbo
    shader.load( "shaderVert.c", "shaderFrag.c" );
    fbo.allocate( 640, 480);
	
	FilteredDepthImage.allocate(640, 480);
	FilteredDepthImage.setUseTexture(true);
	
	// setup the gui
    setupGui();
	
    //	surfaceRenderer=new SurfaceRenderer(640,480,depthProjection,basePlane);
    //	surfaceRenderer->setHeightMapRange(heightMap.getNumEntries(),heightMap.getScalarRangeMin(),heightMap.getScalarRangeMax());
    //	surfaceRenderer->setContourLineDistance(contourlinefactor/10);
    //	surfaceRenderer->setDrawContourLines(false);
	
	kinectgrabber.startThread();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){
    ofSetColor(ofColor::white);
//    fboChessboard.draw(0, 0);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
