#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    
    // kinectgrabber: start
	kinectgrabber.setup();

    // Get projector and kinect width & height
    ofVec2f kinSize = kinectgrabber.getKinectSize();
    kinectResX = kinSize.x;
    kinectResY = kinSize.y;
	projResX =projWindow->getWidth();
	projResY =projWindow->getHeight();
	
	// settings and defaults
	generalState = GENERAL_STATE_CALIBRATION;
	calibrationState  = CALIBRATION_STATE_ROI_DETERMINATION;
    saved = false;
    loaded = false;
    
    // Proj and kinect related variables
	kinectROI = ofRectangle(0, 0, kinectResX, kinectResY);
	
	//Mesh
	meshwidth = 2; // 640 should be dividable by meshwidth-1
	meshheight = 2; // idem with 480
	
	// Setup framefilter variables
    elevationMin=950;
	elevationMax=750;
	int numAveragingSlots=30;
	unsigned int minNumSamples=10;
	unsigned int maxVariance=2;
	float hysteresis=0.1f;
	bool spatialFilter=false;
	gradFieldresolution = 20;
    
    // calibration config
	chessboardSize = 300;
	chessboardX = 5;
    chessboardY = 4;
    fboChessboard.allocate(projResX, projResY, GL_RGBA);
	
//    horizontalMirror = false;//true;
//	verticalMirror = false;//true;
	
    // Setup sandbox boundaries, base plane and kinect clip planes
	basePlaneNormal = ofVec3f(0,0,-1);
	basePlaneNOffset = ofVec3f(0,0,-870);
	nearclip = 750;
	farclip = 950;
	
	// Load colormap
    heightMap.load("HeightColorMap.yml");
    /* Limit the valid elevation range to the extent of the height color map: */
	//if(elevationMin<heightMap.getScalarRangeMin())
	elevationMin=heightMap.getScalarRangeMin();
	//if(elevationMax>heightMap.getScalarRangeMax())
	elevationMax=heightMap.getScalarRangeMax();
	
	kinectgrabber.setupFramefilter(numAveragingSlots, gradFieldresolution, nearclip, farclip, basePlaneNormal, elevationMin, elevationMax);
	kinectgrabber.setKinectROI(kinectROI);
//    setupView();
	//kinectProjectorOutput.load("kinectProjector.yml");
	
    // prepare shaders and fbo
    shader.load( "shaderVert.c", "shaderFrag.c" );
    //fbo.allocate( 640, 480);
	
//	FilteredDepthImage.allocate(640, 480);
//	FilteredDepthImage.setUseTexture(true);
	
	// setup the gui
//    setupGui();
	
    //	surfaceRenderer=new SurfaceRenderer(640,480,depthProjection,basePlane);
    //	surfaceRenderer->setHeightMapRange(heightMap.getNumEntries(),heightMap.getScalarRangeMin(),heightMap.getScalarRangeMax());
    //	surfaceRenderer->setContourLineDistance(contourlinefactor/10);
    //	surfaceRenderer->setDrawContourLines(false);
	
	kinectgrabber.startThread();
}

//--------------------------------------------------------------
void ofApp::addPointPair() {
    int nDepthPoints = 0;
    for (int i=0; i<cvPoints.size(); i++) {
        ofVec3f worldPoint = kinectgrabber.kinect.getWorldCoordinateAt(cvPoints[i].x, cvPoints[i].y);
        if (worldPoint.z > 0)   nDepthPoints++;
    }
    if (nDepthPoints == (chessboardX-1)*(chessboardY-1)) {
        for (int i=0; i<cvPoints.size(); i++) {
            ofVec3f worldPoint = kinectgrabber.kinect.getWorldCoordinateAt(cvPoints[i].x, cvPoints[i].y);
            pairsKinect.push_back(worldPoint);
            pairsProjector.push_back(currentProjectorPoints[i]);
        }
        resultMessage = "Added " + ofToString((chessboardX-1)*(chessboardY-1)) + " points pairs.";
        resultMessageColor = ofColor(0, 255, 0);
    } else {
        resultMessage = "Points not added because not all chessboard\npoints' depth known. Try re-positionining.";
        resultMessageColor = ofColor(255, 0, 0);
    }
}

//--------------------------------------------------------------
void ofApp::update(){
	// Get depth image from kinect grabber
    ofFloatPixels filteredframe;
	if (kinectgrabber.filtered.tryReceive(filteredframe)) {
		FilteredDepthImage.setFromPixels(filteredframe.getData(), kinectResX, kinectResY);
		FilteredDepthImage.updateTexture();
		kinectgrabber.lock();
		kinectgrabber.storedframes -= 1;
		kinectgrabber.unlock();
        
        if (generalState == GENERAL_STATE_SANDBOX) {
            ofVec2f t = ofVec2f(min((float)kinectResX-1,testPoint.x), min((float)kinectResY-1,testPoint.y));
            ofVec3f worldPoint = kinectgrabber.kinect.getWorldCoordinateAt(t.x, t.y);
            ofVec2f projectedPoint = kpt.getProjectedPoint(worldPoint);
            drawTestingPoint(projectedPoint);
        } else if (generalState == GENERAL_STATE_CALIBRATION) {
            // Get color image from kinect grabber
            ofPixels coloredframe;
            if (kinectgrabber.colored.tryReceive(coloredframe)) {
                ///		// If true, `filteredframe` can be used.
                kinectColorImage.setFromPixels(coloredframe);
                
                drawChessboard(ofGetMouseX(), ofGetMouseY(), chessboardSize);
                cvRgbImage = ofxCv::toCv(rgbImage.getPixels());
                cv::Size patternSize = cv::Size(chessboardX-1, chessboardY-1);
                int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
                bool foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
                if(foundChessboard) {
                    cv::Mat gray;
                    cvtColor(cvRgbImage, gray, CV_RGB2GRAY);
                    cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                    drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
                }
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    kinectColorImage.draw(0, 0);
    FilteredDepthImage.draw(10, 490, 320, 240);
    
    ofSetColor(0);
    if (generalState == GENERAL_STATE_SANDBOX) {
        ofDrawBitmapString("Click on the image to test a point in the RGB image.", 340, 510);
        ofDrawBitmapString("The projector should place a green dot on the corresponding point.", 340, 530);
        ofDrawBitmapString("Press the 's' key to save the calibration.", 340, 550);
        if (saved) {
            ofDrawBitmapString("Calibration saved.", 340, 590);
        }
        ofSetColor(255, 0, 0);
        float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 40);
        ofDrawCircle(testPoint.x, testPoint.y, ptSize);
    } else if (generalState == GENERAL_STATE_CALIBRATION) {
        ofDrawBitmapString("Position the chessboard using the mouse.", 340, 510);
        ofDrawBitmapString("Adjust the size of the chessboard using the 'q' and 'w' keys.", 340, 530);
        ofDrawBitmapString("Press the spacebar to save a set of point pairs.", 340, 550);
        ofDrawBitmapString("Press the 'c' key to calibrate.", 340, 570);
        ofSetColor(resultMessageColor);
        ofDrawBitmapString(resultMessage, 340, 610);
        ofSetColor(0);
        ofDrawBitmapString(ofToString(pairsKinect.size())+" point pairs collected.", 340, 630);
    }
    ofSetColor(255);
}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){
    ofSetColor(ofColor::white);
    fboChessboard.draw(0, 0);
}

//--------------------------------------------------------------
void ofApp::drawChessboard(int x, int y, int chessboardSize) {
    float w = chessboardSize / chessboardX;
    float h = chessboardSize / chessboardY;
    currentProjectorPoints.clear();
    fboChessboard.begin();
    ofClear(255, 0);
    ofSetColor(0);
    ofTranslate(x, y);
    for (int j=0; j<chessboardY; j++) {
        for (int i=0; i<chessboardX; i++) {
            int x0 = ofMap(i, 0, chessboardX, 0, chessboardSize);
            int y0 = ofMap(j, 0, chessboardY, 0, chessboardSize);
            if (j>0 && i>0) {
                currentProjectorPoints.push_back(ofVec2f(
                                                         ofMap(x+x0, 0, fboChessboard.getWidth(), 0, 1),
                                                         ofMap(y+y0, 0, fboChessboard.getHeight(), 0, 1)
                                                         ));
            }
            if ((i+j)%2==0) ofDrawRectangle(x0, y0, w, h);
        }
    }
    ofSetColor(255);
    fboChessboard.end();
}

//--------------------------------------------------------------
void ofApp::drawTestingPoint(ofVec2f projectedPoint) {
    float ptSize = ofMap(sin(ofGetFrameNum()*0.1), -1, 1, 3, 40);
    fboChessboard.begin();
    ofBackground(255);
    ofSetColor(0, 255, 0);
    ofDrawCircle(
             ofMap(projectedPoint.x, 0, 1, 0, fboChessboard.getWidth()),
             ofMap(projectedPoint.y, 0, 1, 0, fboChessboard.getHeight()),
             ptSize);
    ofSetColor(255);
    fboChessboard.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key==' '){
        addPointPair();
    } else if (key=='q') {
        chessboardSize -= 20;
    } else if (key=='w') {
        chessboardSize += 20;
    } else if (key=='c') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState  = CALIBRATION_STATE_ROI_DETERMINATION;
        kpt.calibrate(pairsKinect, pairsProjector);
        saved = false;
        loaded = false;
    } else if (key=='t') {
        generalState = GENERAL_STATE_SANDBOX;
//        kpt.calibrate(pairsKinect, pairsProjector);
    }else if (key=='s') {
        kpt.saveCalibration("calibration.xml");
        saved = true;
    } else if (key=='l') {
        kpt.loadCalibration("calibration.xml");
        loaded = true;
    }
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
    if (generalState == GENERAL_STATE_SANDBOX) {
        testPoint.set(min(x, kinectResX-1), min(y, kinectResY-1));
    }
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
