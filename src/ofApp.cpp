#include "ofApp.h"

using namespace ofxCv;
using namespace cv;
using namespace ofxCSG;

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
    calibrated = false;
    
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
	
//    horizontalMirror = false;//true;
//	verticalMirror = false;//true;
	
    // Setup sandbox boundaries, base plane and kinect clip planes
	basePlaneNormal = ofVec3f(0,0,1);
	basePlaneOffset= ofVec3f(0,0,870);
	nearclip = 750;
	farclip = 950;
    basePlaneEq=getPlaneEquation(basePlaneNormal,basePlaneOffset); //homogeneous base plane equation
		
	// Load colormap and set heightmap
    heightMap.load("HeightColorMap.yml");
    /* Limit the valid elevation range to the extent of the height color map: */
	//if(elevationMin<heightMap.getScalarRangeMin())
	elevationMin=heightMap.getScalarRangeMin();
	//if(elevationMax>heightMap.getScalarRangeMax())
	elevationMax=heightMap.getScalarRangeMax();
	setHeightMapRange(heightMap.getNumEntries(),heightMap.getScalarRangeMin(),heightMap.getScalarRangeMax());
    
    // Initialise mesh
//	meshwidth = 2; // 640 should be dividable by meshwidth-1
//	meshheight = 2; // idem with 480
    mesh.clear();
 	for(unsigned int y=0;y<kinectResY;++y)
		for(unsigned int x=0;x<kinectResX;++x)
        {
            mesh.addVertex(ofPoint(float(x)+0.5f,float(y)+0.5f,0.0f)); // make a new vertex
        }
    for(unsigned int y=1;y<kinectResY;++y)
		for(unsigned int x=0;x<kinectResX;++x)
        {
            mesh.addIndex(y*kinectResX+x);
            mesh.addIndex((y-1)*kinectResX+x);
        }
	
	// Load shaders
    elevationShader.load("SurfaceElevationShader.vs", "SurfaceElevationShader.fs" );
    heightMapShader.load("heightMapShader.vs", "heightMapShader.fs");
    
	// Initialize the fbos
    fboProjWindow.allocate(projResX, projResY, GL_RGBA);
    contourLineFramebufferObject.allocate(projResX+1, projResY+1, GL_RGBA);
    
    // Sandbox drawing variables
    drawContourLines = true; // Flag if topographic contour lines are enabled
	contourLineFactor = 0.1f; // Inverse elevation distance between adjacent topographic contour lines
    
	// finish kinectgrabber setup and start the grabber
    kinectgrabber.setupFramefilter(numAveragingSlots, gradFieldresolution, nearclip, farclip, basePlaneNormal, elevationMin, elevationMax, kinectROI);
    kinectgrabber.setMode(generalState, calibrationState);
    kinectWorldMatrix = kinectgrabber.getWorldMatrix();
    cout << "kinectWorldMatrix" << kinectWorldMatrix << endl;
	kinectgrabber.startThread();
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

        if (generalState == GENERAL_STATE_CALIBRATION) {
            // Get color image from kinect grabber
            ofPixels coloredframe;
            if (kinectgrabber.colored.tryReceive(coloredframe)) {
                kinectColorImage.setFromPixels(coloredframe);

                if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
                    ofVec2f t = ofVec2f(min((float)kinectResX-1,testPoint.x), min((float)kinectResY-1,testPoint.y));
                    ofVec3f worldPoint = kinectgrabber.kinect.getWorldCoordinateAt(t.x, t.y);
                    //ofVec3f worldPoint = ofVec3f(t.x, t.y, kinectgrabber.kinect.getDistanceAt(t.x, t.y));
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
                    updateROI();
                }
            }
        }
        else if (generalState == GENERAL_STATE_SANDBOX){
            //Check values for debug
            float maxval, maxval2 = 0.0;
            float minval, minval2 = 100000000.0;
            for (int i = 0; i<640*480; i ++){
                if (filteredframe.getData()[i] > maxval)
                    maxval = filteredframe.getData()[i];
                if (filteredframe.getData()[i] < minval)
                    minval = filteredframe.getData()[i];

                if (FilteredDepthImage.getFloatPixelsRef().getData()[i] > maxval2)
                    maxval2 = FilteredDepthImage.getFloatPixelsRef().getData()[i];
                if (FilteredDepthImage.getFloatPixelsRef().getData()[i] < minval2)
                    minval2 = FilteredDepthImage.getFloatPixelsRef().getData()[i];
            }
            cout << "filtredframe maxval : " << maxval << " filtredframe minval : "<< minval << endl;
            cout << "FilteredDepthImage maxval2 : " << maxval2 << " FilteredDepthImage minval2 : " << minval2 << endl;
            
            drawSandbox();
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    if (generalState == GENERAL_STATE_CALIBRATION) {
        kinectColorImage.draw(0, 0, 640, 480);
        FilteredDepthImage.draw(650, 0, 320, 240);
        
        ofSetColor(0);
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
            ofDrawBitmapStringHighlight("Press the 'r' key to find ROI.", 340, 590);
            ofDrawBitmapStringHighlight("Press the 't' key to switch between performing calibrating and testing calibration.", 340, 610);
            ofDrawBitmapStringHighlight("Press the 'b' key to switch between calibrating and using sandbox.", 340, 630);
            ofDrawBitmapStringHighlight(resultMessage, 340, 650);
            ofDrawBitmapStringHighlight(ofToString(pairsKinect.size())+" point pairs collected.", 340, 670);
        }
        ofSetColor(255);
    }
    else if (generalState == GENERAL_STATE_SANDBOX){
        
    }
}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){ // Main draw call for proj window
    ofSetColor(ofColor::white);
    fboProjWindow.draw(0, 0);
}

//--------------------------------------------------------------
void ofApp::drawChessboard(int x, int y, int chessboardSize) { // Prepare proj window fbo
    float w = chessboardSize / chessboardX;
    float h = chessboardSize / chessboardY;
    currentProjectorPoints.clear();
    fboProjWindow.begin();
    ofClear(255, 0);
    ofSetColor(0);
    ofTranslate(x, y);
    for (int j=0; j<chessboardY; j++) {
        for (int i=0; i<chessboardX; i++) {
            int x0 = ofMap(i, 0, chessboardX, 0, chessboardSize);
            int y0 = ofMap(j, 0, chessboardY, 0, chessboardSize);
            if (j>0 && i>0) {
// Not-normalized (on proj screen)
                currentProjectorPoints.push_back(ofVec2f(x+x0, y+y0));
// Normalized coordinates (between 0 and 1)
//                currentProjectorPoints.push_back(ofVec2f(
//                                                         ofMap(x+x0, 0, fboProjWindow.getWidth(), 0, 1),
//                                                         ofMap(y+y0, 0, fboProjWindow.getHeight(), 0, 1)
//                                                         ));
            }
            if ((i+j)%2==0) ofDrawRectangle(x0, y0, w, h);
        }
    }
    ofSetColor(255);
    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::drawTestingPoint(ofVec2f projectedPoint) { // Prepare proj window fbo
    float ptSize = ofMap(sin(ofGetFrameNum()*0.1), -1, 1, 3, 40);
    fboProjWindow.begin();
    ofBackground(255);
    ofSetColor(0, 255, 0);
// Not-normalized (on proj screen)
    ofDrawCircle(projectedPoint.x, projectedPoint.y, ptSize);
// Normalized coordinates (between 0 and 1)
//    ofDrawCircle(
//             ofMap(projectedPoint.x, 0, 1, 0, fboProjWindow.getWidth()),
//             ofMap(projectedPoint.y, 0, 1, 0, fboProjWindow.getHeight()),
//             ptSize);
    ofSetColor(255);
    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::drawSandbox() { // Prepare proj window fbo
    
//    ofPoint result = computeTransform(kinectROI.getCenter());
    
	/* Check if contour line rendering is enabled: */
	if(drawContourLines)
    {
		/* Run the first rendering pass to create a half-pixel offset texture of surface elevations: */
        //		prepareContourLines();
        //        contourLineFramebufferObject.allocate(800, 600);
    }
    
	/* Bind the single-pass surface shader: */
    fboProjWindow.begin();
    ofClear(255,255,255, 0);

    heightMapShader.begin();
	
    heightMapShader.setUniformTexture( "depthSampler", FilteredDepthImage.getTexture(), 1 ); //"1" means that it is texture 1
    heightMapShader.setUniformMatrix4f("kinectProjMatrix",kinectProjMatrix);
    heightMapShader.setUniformMatrix4f("kinectWorldMatrix",kinectWorldMatrix);
    heightMapShader.setUniform4f("basePlane",basePlaneEq);
    heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
    //    heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 2);
    heightMapShader.setUniform1f("contourLineFactor",contourLineFactor);
    heightMapShader.setUniformTexture("heightColorMapSampler",heightMap.getTexture(), 3);
    
	/* Draw the surface: */
    mesh.draw();
    heightMapShader.end();
    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::prepareContourLines() // Prepare contour line fbo
{
	/*********************************************************************
     Prepare the half-pixel-offset frame buffer for subsequent per-fragment
     Marching Squares contour line extraction.
     *********************************************************************/
	
	/* Adjust the projection matrix to render the corners of the final pixels: */
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX,proj);
	GLdouble xs=GLdouble(800)/GLdouble(801);
	GLdouble ys=GLdouble(600)/GLdouble(601);
	for(int j=0;j<4;++j)
    {
		proj[j*4+0]*=xs;
		proj[j*4+1]*=ys;
    }
	glLoadIdentity();
	glMultMatrixd(proj);
	
	/*********************************************************************
     Render the surface's elevation into the half-pixel offset frame
     buffer.
     *********************************************************************/
	
	/* start the elevation shader and contourLineFramebufferObject: */
    contourLineFramebufferObject.begin();
    ofClear(255,255,255, 0);

	elevationShader.begin();

    elevationShader.setUniformTexture( "depthSampler", FilteredDepthImage.getTexture(), 1 ); //"1" means that it is texture 1
    elevationShader.setUniformMatrix4f("kinectWorldMatrix",kinectWorldMatrix);
    elevationShader.setUniform4f("basePlane",basePlaneEq);
	
	/* Draw the surface: */
    mesh.draw();
	
    elevationShader.end();
    contourLineFramebufferObject.end();
	
	/*********************************************************************
     Restore previous OpenGL state.
     *********************************************************************/
	
	/* Restore the original viewport and projection matrix: */
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
    //	glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
    //
    //	/* Restore the original clear color and frame buffer binding: */
    //	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,currentFrameBuffer);
    //	glClearColor(currentClearColor[0],currentClearColor[1],currentClearColor[2],currentClearColor[3]);
}

//--------------------------------------------------------------
void ofApp::addPointPair() {
    // Add point pair based on kinect world coordinates
    cout << "Adding point pair in kinect world coordinates" << endl;
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
    
    // Add point pair base on kinect camera coordinate (x, y in 640x480, z in calibrated units)
//    cout << "Adding point pair in kinect camera coordinates" << endl;
//    int nDepthPoints = 0;
//    for (int i=0; i<cvPoints.size(); i++) {
//        ofVec3f worldPoint = ofVec3f(cvPoints[i].x, cvPoints[i].y, kinectgrabber.kinect.getDistanceAt(cvPoints[i].x, cvPoints[i].y));
//        if (worldPoint.z > 0)   nDepthPoints++;
//    }
//    if (nDepthPoints == (chessboardX-1)*(chessboardY-1)) {
//        for (int i=0; i<cvPoints.size(); i++) {
//            ofVec3f worldPoint = ofVec3f(cvPoints[i].x, cvPoints[i].y, kinectgrabber.kinect.getDistanceAt(cvPoints[i].x, cvPoints[i].y));
//            pairsKinect.push_back(worldPoint);
//            pairsProjector.push_back(currentProjectorPoints[i]);
//        }
//        resultMessage = "Added " + ofToString((chessboardX-1)*(chessboardY-1)) + " points pairs.";
//        resultMessageColor = ofColor(0, 255, 0);
//    } else {
//        resultMessage = "Points not added because not all chessboard\npoints' depth known. Try re-positionining.";
//        resultMessageColor = ofColor(255, 0, 0);
//    }
//    cout << resultMessage << endl;
//
}

//--------------------------------------------------------------
void ofApp::setHeightMapRange(GLsizei newHeightMapSize,GLfloat newMinElevation,GLfloat newMaxElevation)
{
	/* Calculate the new height map elevation scaling and offset coefficients: */
	GLdouble hms=GLdouble(newHeightMapSize-1)/((newMaxElevation-newMinElevation)*GLdouble(newHeightMapSize));
	GLdouble hmo=0.5/GLdouble(newHeightMapSize)-hms*newMinElevation;
	
	heightMapScale=GLfloat(hms);
	heightMapOffset=GLfloat(hmo);
}

//--------------------------------------------------------------
ofVec2f ofApp::computeTransform(ofPoint vin)
{
    /* Get the vertex' depth image-space z coordinate from the texture: */
    ofVec4f vertexDic=vin;
    vertexDic.z=FilteredDepthImage.getPixelsAsFloats()[((int)vin.x + (int)vin.y*kinectResX)];
    vertexDic.w = 1;
    
    /* Transform the vertex from depth image space to world space: */
    ofVec3f vertexCcxx = kinectgrabber.kinect.getWorldCoordinateAt(vertexDic.x, vertexDic.y, vertexDic.z);
    ofVec4f vertexCc = kinectWorldMatrix*vertexDic*vertexDic.z;
    vertexCc.w = 1;
    
    /* Plug camera-space vertex into the base plane equation: */
    float elevation=basePlaneEq.dot(vertexCc);///vertexCc.w;
    
    /* Transform elevation to height color map texture coordinate: */
//    heightColorMapTexCoord=elevation*heightColorMapTransformation.x+heightColorMapTransformation.y;
    
    /* Transform vertex to clip coordinates: */
    ofVec4f screenPos = kinectProjMatrix*vertexCc;
    ofVec2f projectedPoint(screenPos.x/screenPos.z, screenPos.y/screenPos.z);
    return projectedPoint;
}


//--------------------------------------------------------------
// Find kinect ROI of the sandbox
void ofApp::updateROI(){
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
            kinectProjMatrix = kpt.getProjectionMatrix();
            saved = false;
            loaded = false;
            calibrated = true;
        }
    } else if (key=='r') {
        if (cvPoints.size() == 0) {
            cout << "Error: Chessboard not found on screen !!" << endl;
        } else {
            cout << "Finding ROI" << endl;
            generalState = GENERAL_STATE_CALIBRATION;
            calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
            ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
            kinectgrabber.setMode(generalState, calibrationState);
        }
    } else if (key=='t') {
        generalState = GENERAL_STATE_CALIBRATION;
        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST) {
                calibrationState = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
        }    else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION){
                calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
        }
        cout << "Calibration state: " << calibrationState << endl;
        kinectgrabber.setMode(generalState, calibrationState);
//        kpt.calibrate(pairsKinect, pairsProjector);
    }else if (key=='b') {
        if (generalState == GENERAL_STATE_CALIBRATION) {
            generalState = GENERAL_STATE_SANDBOX;
        }
        else if (generalState == GENERAL_STATE_SANDBOX){
            generalState = GENERAL_STATE_CALIBRATION;
        }
        cout << "General state: " << generalState << endl;
        kinectgrabber.setMode(generalState, calibrationState);
        //        kpt.calibrate(pairsKinect, pairsProjector);
    } else if (key=='s') {
        kpt.saveCalibration("calibration.xml");
        cout << "Calibration saved " << endl;
        saved = true;
    } else if (key=='l') {
        kpt.loadCalibration("calibration.xml");
        cout << "Calibration loaded " << endl;
        kinectProjMatrix = kpt.getProjectionMatrix();
        loaded = true;
        calibrated = true;
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
