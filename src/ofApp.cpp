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
	kinectROI = ofRectangle(0, 0, kinectResX, kinectResY);
	
	// settings and defaults
	generalState = GENERAL_STATE_CALIBRATION;
	calibrationState  = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
    ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
    saved = false;
    loaded = false;
    
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
    kinectgrabber.setMode(generalState, calibrationState);
//    setupView();
	//kinectProjectorOutput.load("kinectProjector.yml");
	
    // prepare shaders and fbo
//    shader.load( "shaderVert.c", "shaderFrag.c" );
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
    cout << "Adding point pair" << endl;
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
    cout << resultMessage << endl;
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

		//Check values for debug
//        float maxval, maxval2 = 0.0;
//        float minval, minval2 = 100000000.0;
//		for (int i = 0; i<640*480; i ++){
//			if (filteredframe.getData()[i] > maxval)
// 				maxval = filteredframe.getData()[i];
//            if (FilteredDepthImage.getFloatPixelsRef().getData()[i] > maxval2)
//                maxval2 = FilteredDepthImage.getFloatPixelsRef().getData()[i];
//			if (filteredframe.getData()[i] < minval)
// 				minval = filteredframe.getData()[i];
//            if (FilteredDepthImage.getFloatPixelsRef().getData()[i] < minval2)
//                minval2 = FilteredDepthImage.getFloatPixelsRef().getData()[i];
//        }
//        cout << "maxval : "<< maxval << " maxval2 : " << maxval2 << " minval : "<< minval << " minval2 : " << minval2 << endl;
        
        if (generalState == GENERAL_STATE_CALIBRATION) {
            // Get color image from kinect grabber
            ofPixels coloredframe;
            if (kinectgrabber.colored.tryReceive(coloredframe)) {
                kinectColorImage.setFromPixels(coloredframe);

                if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
                    ofVec2f t = ofVec2f(min((float)kinectResX-1,testPoint.x), min((float)kinectResY-1,testPoint.y));
                    ofVec3f worldPoint = kinectgrabber.kinect.getWorldCoordinateAt(t.x, t.y);
                    ofVec2f projectedPoint = kpt.getProjectedPoint(worldPoint);
                    drawTestingPoint(projectedPoint);
                }
                else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION) {
                    drawChessboard(ofGetMouseX(), ofGetMouseY(), chessboardSize);
                    cvRgbImage = ofxCv::toCv(kinectColorImage.getPixels());
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
                else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION){
                    // Find kinect ROI of the sandbox
                    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) { // set kinect to max depth range
                        if (cvPoints.size() == 0) {
                            cout << "Error: No points !!" << endl;
                        }
                        cout << "Initiating kinect clip planes" << endl;
                        kinectgrabber.nearclipchannel.send(500);
                        kinectgrabber.farclipchannel.send(4000);
                        ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
                        
                        large = ofPolyline();
                        threshold = 220;
                    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
                        while (threshold < 255){
                            cout << "Increasing threshold : " << threshold << endl;
                            thresholdedImage.setFromPixels(FilteredDepthImage.getPixels());
//                            thresholdedImage.mirror(verticalMirror, horizontalMirror);
                            //cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), highThresh+10, 255, CV_THRESH_TOZERO_INV);
                            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_TOZERO);
                            
                            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true);
                            //contourFinder.findContours(thresholdedImage);
                            //ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
                            
                            ofPolyline small = ofPolyline();
                            for (int i = 0; i < contourFinder.nBlobs; i++) {
                                ofxCvBlob blobContour = contourFinder.blobs[i];
                                if (blobContour.hole) {
                                    //								if(!blobContour.isClosed())
                                    //									blobContour.close();
                                    bool ok = true;
                                    ofPolyline poly = ofPolyline(blobContour.pts);//.getResampledByCount(50);
                                    for (int j = 0; j < cvPoints.size(); j++){ // We only take the 12 first point to speed up process
                                        if (!poly.inside(cvPoints[j].x, cvPoints[j].y)) {
                                            ok = false;
                                            break;
                                        }
                                    }
                                    if (ok) {
                                        cout << "We found a contour lines surroundings the chessboard" << endl;
                                        if (small.size() == 0 || poly.getArea() < small.getArea())
                                            cout << "We take the smallest contour line surroundings the chessboard at a given threshold level" << endl;
                                            small = poly;
                                    }
                                }
                            }
                            if (large.getArea() < small.getArea())
                                cout << "We take the largest contour line surroundings the chessboard at all threshold level" << endl;
                                large = small;
                            threshold+=1;
                        } //else {
                        kinectROI = large.getBoundingBox();
//                        if (horizontalMirror) {
//                            kinectROI.x = 640 -kinectROI.x;
//                            kinectROI.width = -kinectROI.width;
//                        }
//                        if (verticalMirror) {
//                            kinectROI.y = 480 -kinectROI.y;
//                            kinectROI.height = -kinectROI.height;
//                        }
                        kinectROI.standardize();
                        cout << kinectROI << endl;
                        // We are finished, set back kinect depth range and update ROI
                        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
                        kinectgrabber.setKinectROI(kinectROI);
                        kinectgrabber.nearclipchannel.send(nearclip);
                        kinectgrabber.farclipchannel.send(farclip);
                        //}
                    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
                        generalState = GENERAL_STATE_CALIBRATION;
                        calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
                    }
                }
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    kinectColorImage.draw(0, 0, 640, 480);
    FilteredDepthImage.draw(650, 0, 320, 240);
    
    ofSetColor(0);
    if (generalState == GENERAL_STATE_CALIBRATION) {
        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
            ofDrawBitmapStringHighlight("Click on the image to test a point in the RGB image.", 340, 510);
            ofDrawBitmapStringHighlight("The projector should place a green dot on the corresponding point.", 340, 530);
            ofDrawBitmapStringHighlight("Press the 's' key to save the calibration.", 340, 550);
            if (saved) {
                ofDrawBitmapStringHighlight("Calibration saved.", 340, 590);
            }
            ofSetColor(255, 0, 0);
            float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 40);
            ofDrawCircle(testPoint.x, testPoint.y, ptSize);
        } else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION || calibrationState == CALIBRATION_STATE_ROI_DETERMINATION) {
            ofNoFill();
            ofSetColor(255);
            ofDrawRectangle(kinectROI);
            ofFill();
            ofDrawBitmapStringHighlight("Position the chessboard using the mouse.", 340, 510);
            ofDrawBitmapStringHighlight("Adjust the size of the chessboard using the 'q' and 'w' keys.", 340, 530);
            ofDrawBitmapStringHighlight("Press the spacebar to save a set of point pairs.", 340, 550);
            ofDrawBitmapStringHighlight("Press the 'c' key to perform calibration.", 340, 570);
            ofDrawBitmapStringHighlight("Press the 't' key to switch between calibrating and testing calibration.", 340, 590);
            ofDrawBitmapStringHighlight("Press the 'r' key to find ROI.", 340, 610);
            ofDrawBitmapStringHighlight(resultMessage, 340, 630);
            ofDrawBitmapStringHighlight(ofToString(pairsKinect.size())+" point pairs collected.", 340, 650);
        }
        ofSetColor(255);
    }
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
        if (pairsKinect.size() == 0) {
            cout << "Error: No points acquired !!" << endl;
        } else {
            cout << "calibrating" << endl;
            kpt.calibrate(pairsKinect, pairsProjector);
            saved = false;
            loaded = false;
        }
    } else if (key=='r') {
        if (cvPoints.size() == 0) {
            cout << "Error: Chessboard not found on screen !!" << endl;
        } else {
            cout << "Finding ROI" << endl;
            generalState = GENERAL_STATE_CALIBRATION;
            calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
            ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        }
    } else if (key=='t') {
        generalState = GENERAL_STATE_CALIBRATION;
        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST) {
                calibrationState = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
        }    else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION){
                calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
        }
        cout << "Calibration state: " << calibrationState << endl;
//        kpt.calibrate(pairsKinect, pairsProjector);
    }else if (key=='s') {
        kpt.saveCalibration("calibration.xml");
        cout << "Calibration saved " << endl;
        saved = true;
    } else if (key=='l') {
        kpt.loadCalibration("calibration.xml");
        cout << "Calibration loaded " << endl;
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
    if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_CALIBRATION_TEST) {
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
