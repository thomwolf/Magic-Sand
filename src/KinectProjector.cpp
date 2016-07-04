//
//  KinectProjector.cpp
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#include "KinectProjector.h"

using namespace ofxCSG;

void KinectProjector::setup(ofVec2f sprojRes){
	// settings and defaults
    calibrated = false;
    calibrating = false;
    baseplaneUpdated = false;
    ROIUpdated = false;
    imageStabilized = false;

    // instantiate the modal window //
    modal = make_shared<ofxModalAlert>();
    modal->addListener(this, &KinectProjector::onModalEvent);
    //    modal = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT );
    //    modal->addLabel(modaltext);
    //    modal->addButton("Ok");
    //    modal->addButton("Cancel");
    //    // hide the modal box //
    //    showModal = false;
    //    modal->setVisible(showModal);
    //    modal->onButtonEvent(this, &ofApp::onModalButtonEvent);

    // calibration chessboard config
	chessboardSize = 300;
	chessboardX = 5;
    chessboardY = 4;
    
    // 	gradFieldresolution
	gradFieldresolution = 10;
    arrowLength = 25;
	
    // Setup default base plane
	basePlaneNormalBack = ofVec3f(0,0,1); // This is our default baseplane normal
	basePlaneOffsetBack= ofVec3f(0,0,870); // This is our default baseplane offset
    basePlaneNormal = basePlaneNormalBack;
    basePlaneOffset = basePlaneOffsetBack;
    basePlaneEq=getPlaneEquation(basePlaneOffset,basePlaneNormal);
    maxOffset = 0;
    maxOffsetSafeRange = 50; // Range above the autocalib measured max offset

    // kinectgrabber: start
	kinectgrabber.setup();
    
    // Get projector and kinect width & height
    projRes = sprojRes;
    kinectRes = kinectgrabber.getKinectSize();
	kinectROI = ofRectangle(0, 0, kinectRes.x, kinectRes.y);
    
    // Initialize the fbos and images
    FilteredDepthImage.allocate(kinectRes.x, kinectRes.y);
    kinectColorImage.allocate(kinectRes.x, kinectRes.y);
    thresholdedImage.allocate(kinectRes.x, kinectRes.y);
    Dptimg.allocate(20, 20); // Small detailed ROI
    
    //Try to load calibration file if possible
    if (kpt.loadCalibration("calibration.xml"))
    {
        ofLogVerbose("GreatSand") << "setup(): Calibration loaded " ;
        kinectProjMatrix = kpt.getProjectionMatrix();
        ofLogVerbose("GreatSand") << "setup(): kinectProjMatrix: " << kinectProjMatrix ;
        calibrated = true;
//        generalState = GENERAL_STATE_SANDBOX;
//        updateMode();
    } else {
//        generalState = GENERAL_STATE_CALIBRATION;
//        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
//        initialisationState = INITIALISATION_STATE_ROI_DETERMINATION;
//        ROICalibState = ROI_CALIBRATION_STATE_INIT;
//        updateMode();
        ofLogVerbose("GreatSand") << "setup(): Calibration could not be loaded" ;
    }
    
    //Try to load settings file if possible
    if (loadSettings("settings.xml"))
    {
        ofLogVerbose("GreatSand") << "setup(): Settings loaded " ;
        ROICalibState = ROI_CALIBRATION_STATE_DONE;
    } else {
        ofLogVerbose("GreatSand") << "setup(): Settings could not be loaded " ;
    }
    
	// finish kinectgrabber setup and start the grabber
    kinectgrabber.setupFramefilter(gradFieldresolution, maxOffset, kinectROI);
    kinectWorldMatrix = kinectgrabber.getWorldMatrix();
    ofLogVerbose("GreatSand") << "setup(): kinectWorldMatrix: " << kinectWorldMatrix ;
    
    // Setup gradient field
    setupGradientField();
    
    fboProjWindow.allocate(projRes.x, projRes.y, GL_RGBA);
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();
    
    kinectgrabber.start(); // Start the acquisition
}

void KinectProjector::setupGradientField(){
    gradFieldcols = kinectRes.x / gradFieldresolution;
    gradFieldrows = kinectRes.y / gradFieldresolution;
    
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
}

void KinectProjector::update(){
    // Get depth image from kinect grabber
    ofFloatPixels filteredframe;
    if (kinectgrabber.filtered.tryReceive(filteredframe)) {
        FilteredDepthImage.setFromPixels(filteredframe.getData(), kinectRes.x, kinectRes.y);
        FilteredDepthImage.updateTexture();
        
        // Get color image from kinect grabber
        ofPixels coloredframe;
        if (kinectgrabber.colored.tryReceive(coloredframe)) {
            kinectColorImage.setFromPixels(coloredframe);
        }
        
        // Update grabber stored frame number
        kinectgrabber.lock();
        kinectgrabber.decStoredframes();
        kinectgrabber.unlock();
        
        // Is the depth image stabilized
        if (kinectgrabber.isImageStabilized())
            imageStabilized = true;
        if (calibrating)
            updateCalibration();
    }
}

void KinectProjector::updateCalibration(){
    if (calibrationState == CALIBRATION_STATE_FULL_AUTO_CALIBRATION){
        updateFullAutoCalibration();
    } else if (calibrationState == CALIBRATION_STATE_ROI_AUTO_DETERMINATION){
        updateROIAutoCalibration();
    } else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_DETERMINATION){
        updateROIManualCalibration();
    } else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_AUTO_CALIBRATION){
        updateProjKinectAutoCalibration();
    }else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_MANUAL_CALIBRATION) {
        updateProjKinectManualCalibration();
    }
}

void KinectProjector::updateFullAutoCalibration(){
    if (fullCalibState == FULL_CALIBRATION_STATE_ROI_DETERMINATION){
        updateROIAutoCalibration();
        if (ROICalibState == ROI_CALIBRATION_STATE_DONE) {
            fullCalibState = FULL_CALIBRATION_STATE_AUTOCALIB;
            autoCalibState = AUTOCALIB_STATE_INIT_FIRST_PLANE;
            calibrated = false;
            calibrating = true;
        }
    } else if (fullCalibState == FULL_CALIBRATION_STATE_AUTOCALIB){
        updateProjKinectAutoCalibration();
        if (autoCalibState == AUTOCALIB_STATE_DONE){
            fullCalibState = FULL_CALIBRATION_STATE_DONE;
        }
    }
}

void KinectProjector::updateROIAutoCalibration(){
    //updateROIFromColorImage();
    updateROIFromDepthImage();
}

void KinectProjector::updateROIFromColorImage(){
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) { // set kinect to max depth range
        ROICalibState = ROI_CALIBRATION_STATE_MOVE_UP;
        large = ofPolyline();
        threshold = 90;
        
    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
        while (threshold < 255){
            kinectColorImage.setROI(0, 0, kinectRes.x, kinectRes.y);
            thresholdedImage = kinectColorImage;
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_BINARY_INV);
            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true);
            ofPolyline small = ofPolyline();
            for (int i = 0; i < contourFinder.nBlobs; i++) {
                ofxCvBlob blobContour = contourFinder.blobs[i];
                if (blobContour.hole) {
                    ofPolyline poly = ofPolyline(blobContour.pts);
                    if (poly.inside(kinectRes.x/2, kinectRes.y/2))
                    {
                        if (small.size() == 0 || poly.getArea() < small.getArea()) {
                            small = poly;
                        }
                    }
                }
            }
            ofLogVerbose("GreatSand") << "updateROIFromColorImage(): small.getArea(): " << small.getArea() ;
            ofLogVerbose("GreatSand") << "updateROIFromColorImage(): large.getArea(): " << large.getArea() ;
            if (large.getArea() < small.getArea())
            {
                ofLogVerbose("GreatSand") << "updateROIFromColorImage(): We take the largest contour line surroundings the center of the screen at all threshold level" ;
                large = small;
            }
            threshold+=1;
        }
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        ofLogVerbose("GreatSand") << "updateROIFromColorImage(): kinectROI : " << kinectROI ;
        ROICalibState = ROI_CALIBRATION_STATE_DONE;
        updateKinectGrabberROI();
    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
    }
}

void KinectProjector::updateROIFromDepthImage(){
    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
        kinectROI = ofRectangle(0, 0, kinectRes.x, kinectRes.y);
        updateKinectGrabberROI();
        modaltext = "Detecting sand area: Please wait...";
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_INIT: Set maximal ROI for kinect & Wait for kinectgrabber to reset buffers" ;
        
        while (kinectgrabber.isFirstImageReady()){
        } // Wait for kinectgrabber to reset buffers
        
        imageStabilized = false; // Now we waite for a clean new depth frame
        ROICalibState = ROI_CALIBRATION_STATE_READY_TO_MOVE_UP;
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_INIT: Wait for clean new frame" ;
        
    } else if (ROICalibState == ROI_CALIBRATION_STATE_READY_TO_MOVE_UP) {
        if (imageStabilized)
        {
            ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_READY_TO_MOVE_UP: got a stable depth image" ;
            ROICalibState = ROI_CALIBRATION_STATE_MOVE_UP;
            large = ofPolyline();
            ofxCvFloatImage temp;
            temp.setFromPixels(FilteredDepthImage.getFloatPixelsRef().getData(), kinectRes.x, kinectRes.y);
            temp.setNativeScale(FilteredDepthImage.getNativeScaleMin(), FilteredDepthImage.getNativeScaleMax());
            temp.convertToRange(0, 1);
            thresholdedImage.setFromPixels(temp.getFloatPixelsRef());
        }
        threshold = 0; // We go from the higher distance to the kinect (lower position) to the lower distance
    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
        while (threshold < 255){
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), 255-threshold, 255, CV_THRESH_TOZERO_INV);
            thresholdedImage.updateTexture();
            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true, false);
            ofPolyline small = ofPolyline();
            for (int i = 0; i < contourFinder.nBlobs; i++) {
                ofxCvBlob blobContour = contourFinder.blobs[i];
                if (blobContour.hole) {
                    ofPolyline poly = ofPolyline(blobContour.pts);
                    if (poly.inside(kinectRes.x/2, kinectRes.y/2))
                    {
                        if (small.size() == 0 || poly.getArea() < small.getArea()) {
                            small = poly;
                        }
                    }
                }
            }
            if (large.getArea() < small.getArea())
            {
                ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): updating ROI" ;
                large = small;
            }
            threshold+=1;
        }
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): final kinectROI : " << kinectROI ;
        ROICalibState = ROI_CALIBRATION_STATE_DONE;
        updateKinectGrabberROI();
        modaltext = "Detecting sand area: Sand area successfully detected.";
    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
    }
}

void KinectProjector::updateROIManualCalibration(){
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    
    if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
        kinectROIManualCalib.setSize(ofGetMouseX()-kinectROIManualCalib.x,ofGetMouseY()-kinectROIManualCalib.y);
    }
    
    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
        resultMessage = "Please click on first ROI corner";
    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
        resultMessage = "Please click on second ROI corner";
    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
        resultMessage = "Manual ROI update done";
    }
}

void KinectProjector::updateProjKinectAutoCalibration(){
    if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE){
        fboProjWindow.begin();
        ofBackground(255);
        fboProjWindow.end();
        updateROIAutoCalibration();
        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        modaltext = "Kinect/projector calibration: Please flatten the sand surface carefully before starting the calibration.";
    } else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT){
        if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
            if (kinectROI.getArea() == 0)
            {
                modaltext = "The sand region has not been defined. Please detect or define manually the sand region.";
                autoCalibState = AUTOCALIB_STATE_DONE;
            } else {
                modaltext = "Kinect/projector calibration: Computing base plane, please wait.";
                updateBasePlane(); // Find base plane
                autoCalibPts = new ofPoint[10];
                float cs = 2*chessboardSize/3;
                float css = 3*chessboardSize/4;
                ofPoint sc = ofPoint(projRes.x/2,projRes.y/2);
                
                // Prepare 10 locations for the calibration chessboard
                autoCalibPts[0] = ofPoint(cs,cs)-sc;
                autoCalibPts[1] = ofPoint(projRes.x-cs,cs)-sc;
                autoCalibPts[2] = ofPoint(projRes.x-cs,projRes.y-cs)-sc;
                autoCalibPts[3] = ofPoint(cs,projRes.y-cs)-sc;
                autoCalibPts[4] = ofPoint(projRes.x/2+cs,projRes.y/2)-sc;
                autoCalibPts[5] = ofPoint(css,css)-sc;
                autoCalibPts[6] = ofPoint(projRes.x-css,css)-sc;
                autoCalibPts[7] = ofPoint(projRes.x-css,projRes.y-css)-sc;
                autoCalibPts[8] = ofPoint(css,projRes.y-css)-sc;
                autoCalibPts[9] = ofPoint(projRes.x/2-cs,projRes.y/2)-sc;
                currentCalibPts = 0;
                cleared = false;
                upframe = false;
                trials = 0;
                autoCalibState = AUTOCALIB_STATE_NEXT_POINT;
            }
        } else {
            updateROIAutoSetup();
        }
    } else if (autoCalibState == AUTOCALIB_STATE_NEXT_POINT){
        if (currentCalibPts < 5 || (upframe && currentCalibPts < 10)) {
            modaltext = "Kinect/projector calibration: Acquiring calibration points, please wait.";
            cvRgbImage = ofxCv::toCv(kinectColorImage.getPixels());
            cv::Size patternSize = cv::Size(chessboardX-1, chessboardY-1);
            int chessFlags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
            bool foundChessboard = findChessboardCorners(cvRgbImage, patternSize, cvPoints, chessFlags);
            if(foundChessboard) {
                if (cleared) { // We have previously detected a cleared screen <- Be sure that we don't acquire several times the same chessboard
                    cv::Mat gray;
                    cvtColor(cvRgbImage, gray, CV_RGB2GRAY);
                    cornerSubPix(gray, cvPoints, cv::Size(11, 11), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                    drawChessboardCorners(cvRgbImage, patternSize, cv::Mat(cvPoints), foundChessboard);
                    ofLogVerbose("GreatSand") << "autoCalib(): Chessboard found for point :" << currentCalibPts ;
                    ofLogVerbose("GreatSand") << "autoCalib(): Adding point pair" ;
                    addPointPair();
                    
                    fboProjWindow.begin(); // Clear projector
                    ofBackground(255);
                    fboProjWindow.end();
                    cleared = false;
                    trials = 0;
                    currentCalibPts++;
                }
            } else {
                if (cleared == false) {
                    ofLogVerbose("GreatSand") << "autoCalib(): Clear screen found, drawing next chessboard" ;
                    cleared = true; // The cleared fbo screen was seen by the kinect
                    ofPoint dispPt = ofPoint(projRes.x/2,projRes.y/2)+autoCalibPts[currentCalibPts]; // Compute next chessboard position
                    drawChessboard(dispPt.x, dispPt.y, chessboardSize); // We can now draw the next chess board
                } else {
                    // We cannot find the chessboard
                    trials++;
                    ofLogVerbose("GreatSand") << "autoCalib(): Chessboard not found on trial : " << trials ;
                    if (trials >10) {
                        // Move the chessboard closer to the center of the screen
                        modaltext = "Kinect/projector calibration: Chessboard not found, moving chessboard. Please wait.";
                        ofLogVerbose("GreatSand") << "autoCalib(): Chessboard could not be found moving chessboard closer to center " ;
                        autoCalibPts[currentCalibPts] = 3*autoCalibPts[currentCalibPts]/4;
                        fboProjWindow.begin(); // Clear projector
                        ofBackground(255);
                        fboProjWindow.end();
                        cleared = false;
                        trials = 0;
                    }
                }
            }
        } else {
            if (upframe) { // We are done
                updateMaxOffset(); // Find max offset
                autoCalibState = AUTOCALIB_STATE_COMPUTE;
            } else { // We ask for higher points
                modaltext = "Please put a board over the sandbox so we can acquire higher calibration points.";
            }
        }
    } else if (autoCalibState == AUTOCALIB_STATE_COMPUTE){
        if (pairsKinect.size() == 0) {
            ofLogVerbose("GreatSand") << "autoCalib(): Error: No points acquired !!" ;
        } else {
            ofLogVerbose("GreatSand") << "autoCalib(): Calibrating" ;
            kpt.calibrate(pairsKinect, pairsProjector);
            kinectProjMatrix = kpt.getProjectionMatrix();
            calibrated = true;
            modaltext = "Kinect/projector calibration: Calibration successfull.";
            saveCalibrationAndSettings();
        }
        autoCalibState = AUTOCALIB_STATE_DONE;
    } else if (autoCalibState == AUTOCALIB_STATE_DONE){
    }
}

void KinectProjector::updateProjKinectManualCalibration(){
    // Draw a Chessboard
    drawChessboard(ofGetMouseX(), ofGetMouseY(), chessboardSize);
    // Try to find the chess board on the kinect color image
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

void KinectProjector::updateBasePlane(){
    ofRectangle smallROI = getKinectROI();
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    ofLogVerbose("GreatSand") << "updateBasePlane(): smallROI: " << smallROI ;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    ofLogVerbose("GreatSand") << "updateBasePlane(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    if (sw*sh == 0) {
        ofLogVerbose("GreatSand") << "updateBasePlane(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    ofVec4f pt;
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("GreatSand") << "updateBasePlane(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = ofVec3f(kinectCoordToWorldCoord(x+sl, y+st));
        }
    }
    ofLogVerbose("GreatSand") << "updateBasePlane(): Computing plane from points" ;
    basePlaneEq = plane_from_points(points, sw*sh);
    basePlaneNormal = ofVec3f(basePlaneEq);
    basePlaneOffset = ofVec3f(0,0,-basePlaneEq.w);
    basePlaneUpdated = true;
}

void KinectProjector::updateMaxOffset(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    ofLogVerbose("GreatSand") << "updateMaxOffset(): smallROI: " << smallROI ;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    ofLogVerbose("GreatSand") << "updateMaxOffset(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    if (sw*sh == 0) {
        ofLogVerbose("GreatSand") << "updateMaxOffset(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    ofVec4f pt;
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("GreatSand") << "updateMaxOffset(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = ofVec3f(kinectCoordToWorldCoord(x+sl, y+st));//vertexCcvertexCc;
        }
    }
    ofLogVerbose("GreatSand") << "updateMaxOffset(): Computing plane from points" ;
    ofVec4f eqoff = plane_from_points(points, sw*sh);
    maxOffset = -eqoff.w-maxOffsetSafeRange;
    // Update max Offset
    ofLogVerbose("GreatSand") << "updateMaxOffset(): maxOffset" << maxOffset ;
    kinectgrabber.setMaxOffset(maxOffset);
}

void KinectProjector::addPointPair() {
    ofLogVerbose("GreatSand") << "addPointPair(): Adding point pair in kinect world coordinates" ;
    int nDepthPoints = 0;
    for (int i=0; i<cvPoints.size(); i++) {
        ofVec3f worldPoint = kinectCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
        if (worldPoint.z > 0)   nDepthPoints++;
    }
    if (nDepthPoints == (chessboardX-1)*(chessboardY-1)) {
        for (int i=0; i<cvPoints.size(); i++) {
            ofVec3f worldPoint = kinectCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
            worldPoint.z = worldPoint.z;
            pairsKinect.push_back(worldPoint);
            pairsProjector.push_back(currentProjectorPoints[i]);
        }
        resultMessage = "addPointPair(): Added " + ofToString((chessboardX-1)*(chessboardY-1)) + " points pairs.";
        resultMessageColor = ofColor(0, 255, 0);
    } else {
        resultMessage = "addPointPair(): Points not added because not all chessboard\npoints' depth known. Try re-positionining.";
        resultMessageColor = ofColor(255, 0, 0);
    }
    ofLogVerbose("GreatSand") << resultMessage ;
}

void KinectProjector::updateKinectGrabberROI(){
    saveCalibrationAndSettings();
    ROIUpdated = true;
#if __cplusplus>=201103
    kinectgrabber.ROIchannel.send(std::move(kinectROI));
#else
    kinectgrabber.ROIchannel.send(kinectROI);
#endif
}

void KinectProjector::drawProjectorWindow(){
    fboProjWindow.draw(0,0);
}

void KinectProjector::drawMainWindow(){
//    fboProjWindow.draw();
}

void KinectProjector::drawChessboard(int x, int y, int chessboardSize) {
    fboProjWindow.begin();
    // Draw the calibration chess board on the projector window
    float w = chessboardSize / chessboardX;
    float h = chessboardSize / chessboardY;
    
    float xf = x-chessboardSize/2; // x and y are chess board center size
    float yf = y-chessboardSize/2;
    
    currentProjectorPoints.clear();
    
    ofClear(255, 0);
    ofSetColor(0);
    ofTranslate(xf, yf);
    for (int j=0; j<chessboardY; j++) {
        for (int i=0; i<chessboardX; i++) {
            int x0 = ofMap(i, 0, chessboardX, 0, chessboardSize);
            int y0 = ofMap(j, 0, chessboardY, 0, chessboardSize);
            if (j>0 && i>0) {
                currentProjectorPoints.push_back(ofVec2f(xf+x0, yf+y0));
            }
            if ((i+j)%2==0) ofDrawRectangle(x0, y0, w, h);
        }
    }
    ofSetColor(255);
    fboProjWindow.end();
}

void KinectProjector::drawFlowField()
{
    float maxsize = 0;
    for(int rowPos=0; rowPos< gradFieldrows ; rowPos++)
    {
        for(int colPos=0; colPos< gradFieldcols ; colPos++)
        {
            float x = colPos*gradFieldresolution + gradFieldresolution/2;
            float y = rowPos*gradFieldresolution  + gradFieldresolution/2;
            ofVec2f projectedPoint = kinectCoordToProjCoord(x, y);
            ofVec2f v2 = gradField[colPos + rowPos * gradFieldcols];
            if (v2.length() > maxsize)
                maxsize = v2.length();
            v2 *= arrowLength;
            drawArrow(projectedPoint, v2);
        }
    }
}

void KinectProjector::drawArrow(ofVec2f projectedPoint, ofVec2f v1)
{
    ofFill();
    ofPushMatrix();
    ofTranslate(projectedPoint);
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, v1.x, v1.y);
    ofDrawCircle(v1.x, v1.y, 5);
    ofPopMatrix();
}

void KinectProjector::updateNativeScale(float scaleMin, float scaleMax){
    FilteredDepthImage.setNativeScale(scaleMin, scaleMax);
}

ofVec2f KinectProjector::kinectCoordToProjCoord(float x, float y) // x, y in kinect pixel coord
{
    return worldCoordToProjCoord(kinectCoordToWorldCoord(x, y));
}

ofVec2f KinectProjector::worldCoordToProjCoord(ofVec3f vin)
{
    ofVec4f wc = vin;
    wc.w = 1;
    ofVec4f screenPos = kinectProjMatrix*wc;
    ofVec2f projectedPoint(screenPos.x/screenPos.z, screenPos.y/screenPos.z);
    return projectedPoint;
}

ofVec3f KinectProjector::kinectCoordToWorldCoord(float x, float y) // x, y in kinect pixel coord
{
    ofVec4f kc = ofVec2f(x, y);
    kc.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)(y*kinectRes.x+x)];
    kc.w = 1;
    ofVec4f wc = kinectWorldMatrix*kc*kc.z;
    return ofVec3f(wc);
}

float KinectProjector::elevationAtKinectCoord(float x, float y) // x, y in kinect pixel coordinate
{
    ofVec4f wc = kinectCoordToWorldCoord(x, y);
    float elevation = -basePlaneEq.dot(wc);
    return elevation;
}

float KinectProjector::elevationToKinectDepth(float elevation, float x, float y) // x, y in kinect pixel coordinate
{
    ofVec4f wc = kinectCoordToWorldCoord(x, y);
    wc.z = 0;
    wc.w = 1;
    float kinectDepth = -(basePlaneEq.dot(wc)+elevation)/basePlaneEq.z;
    return kinectDepth;
}

ofVec2f gradientAtKinectCoord(float x, float y){
    return gradField[(int)(x/gradFieldresolution+gradFieldcols*y/gradFieldresolution)];
}

void KinectProjector::setupGui(){
    // instantiate and position the gui //
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
    
    // add some components //
    //    gui->addTextInput("message", "# open frameworks #");
    
    gui->addFRM();
    gui->addBreak();
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* sealevelFolder = gui->addFolder("Sea level", ofColor::blueSteel);
    sealevelFolder->addSlider("Tilt X", -30, 30, 0);
    sealevelFolder->addSlider("Tilt Y", -30, 30, 0);
    sealevelFolder->addSlider("Vertical offset", heightMap.getScalarRangeMin()*.5, heightMap.getScalarRangeMax()*.5, 0);
    sealevelFolder->addButton("Reset sea level");
    sealevelFolder->expand();
    
    gui->addBreak();
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* displayFolder = gui->addFolder("Depth frame filtering", ofColor::orangeRed);
    displayFolder->addSlider("Highest detection\nlevel", 0, 100, maxOffset);
    displayFolder->addToggle("Spatial filtering", true);
    displayFolder->addToggle("Quick reaction (follow hands)", true);
    displayFolder->addSlider("# of averaging slots", 0, 40, 30)->setPrecision(0);
    displayFolder->expand();
    
    gui->addBreak();
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* calibrationFolder = gui->addFolder("Calibration", ofColor::purple);
    calibrationFolder->addButton("Full Calibration");
    calibrationFolder->addButton("Automatically detect sand region");
    calibrationFolder->addButton("Manually define sand region");
    calibrationFolder->addButton("Automatically calibrate kinect & projector");
    calibrationFolder->addButton("Manually calibrate kinect & projector");
    calibrationFolder->expand();
    
    gui->addBreak();
    
    // adding the optional header allows you to drag the gui around //
    gui->addHeader(":: Settings ::");
    
    // adding the optional footer allows you to collapse/expand the gui //
    gui->addFooter();
    
    // once the gui has been assembled, register callbacks to listen for component specific events //
    gui->onButtonEvent(this, &KinectProjector::onButtonEvent);
    gui->onToggleEvent(this, &KinectProjector::onToggleEvent);
    gui->onSliderEvent(this, &KinectProjector::onSliderEvent);
    
    //    gui->setTheme(new ofxDatGuiThemeSmoke());
    
    //    gui->onTextInputEvent(this, &ofApp::onTextInputEvent);
    //    gui->on2dPadEvent(this, &ofApp::on2dPadEvent);
    //    gui->onDropdownEvent(this, &ofApp::onDropdownEvent);
    //    gui->onColorPickerEvent(this, &ofApp::onColorPickerEvent);
    //    gui->onMatrixEvent(this, &ofApp::onMatrixEvent);
    
    //    gui->setOpacity(gui->getSlider("datgui opacity")->getScale());
    gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    
    // finally let's load up the stock themes, press spacebar to cycle through them //
    themes = {  new ofxDatGuiTheme(true),
        new ofxDatGuiThemeCalib(),
        new ofxDatGuiThemeSmoke(),
        new ofxDatGuiThemeWireframe(),
        new ofxDatGuiThemeMidnight(),
        new ofxDatGuiThemeAqua(),
        new ofxDatGuiThemeCharcoal(),
        new ofxDatGuiThemeAutumn(),
        new ofxDatGuiThemeCandy()};
    tIndex = 0;
}
void KinectProjector::onModalEvent(ofxModalEvent e){
    if (e.type == ofxModalEvent::SHOWN){
        cout << "modal window is open" << endl;
    }   else if (e.type == ofxModalEvent::HIDDEN){
        cout << "modal window is closed" << endl;
    }   else if (e.type == ofxModalEvent::CANCEL){
        if (calibrated){
            generalState = GENERAL_STATE_SANDBOX;
        } else {
            generalState = GENERAL_STATE_NO_CALIBRATION;
        }
        ofLogVerbose("GreatSand") << "Modal cancel button pressed: Aborting" ;
        updateMode();
        cout << "cancel button was selected" << endl;
    }   else if (e.type == ofxModalEvent::CONFIRM){
        if (generalState == GENERAL_STATE_CALIBRATION){
            if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
            {
                addPointPair();
            } else if (calibrationState == CALIBRATION_STATE_AUTOCALIB){
                if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE)
                {
                    autoCalibState = AUTOCALIB_STATE_INIT_POINT;
                } else if (autoCalibState == AUTOCALIB_STATE_NEXT_POINT) {
                    if (!upframe)
                        upframe = true;
                } else if (autoCalibState == AUTOCALIB_STATE_DONE){
                }
            }
        }
        cout << "confirm button was selected" << endl;
    }
}

void KinectProjector::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Full Calibration")) {
        calibrating = true;
        calibrationState = CALIBRATION_STATE_FULL_AUTO_CALIBRATION;
        fullCalibState = FULL_CALIBRATION_STATE_ROI_DETERMINATION;
        ROICalibState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Starting full calibration" ;
        //        showModal = true;
    } else if (e.target->is("Automatically detect sand region")) {
        calibrating = true;
        calibrationState = CALIBRATION_STATE_ROI_AUTO_DETERMINATION;
        ROICalibState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Finding ROI" ;
        //        showModal = true;
    } else if (e.target->is("Manually define sand region")){
        calibrating = true;
        calibrationState = CALIBRATION_STATE_ROI_MANUAL_DETERMINATION;
        ROICalibState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Updating ROI Manually" ;
        //        showModal = true;
    } else if (e.target->is("Automatically calibrate kinect & projector")){
        calibrating = true;
        calibrationState = CALIBRATION_STATE_PROJ_KINECT_AUTO_CALIBRATION;
        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Starting autocalib" ;
        //        showModal = true;
        //TODO: Fix manual calibration
    } else if (e.target->is("Manually calibrate kinect & projector")) {
        calibrating = true;
        calibrationState = CALIBRATION_STATE_PROJ_KINECT_MANUAL_CALIBRATION;
        ROICalibState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Manual calibration -- Not working now" ;
        //        showModal = true;
    } else if (e.target->is("Reset sea level")){
        basePlaneNormal = basePlaneNormalBack;
        basePlaneOffset.z = basePlaneOffsetBack.z;
        gui->getSlider("Tilt X")->setValue(0);
        gui->getSlider("Tilt Y")->setValue(0);
        gui->getSlider("Vertical offset")->setValue(0);
        setRangesAndBasePlaneEquation();
    }
}

void KinectProjector::onToggleEvent(ofxDatGuiToggleEvent e){
    if (e.target->is("Spatial filtering")) {
        kinectgrabber.setSpatialFiltering(e.checked);
    }else if (e.target->is("Quick reaction (follow hands)")) {
        kinectgrabber.setFollowBigChange(e.checked);
    }
}

void KinectProjector::onSliderEvent(ofxDatGuiSliderEvent e){
    if (e.target->is("Tilt X") || e.target->is("Tilt Y")) {
        basePlaneNormal = basePlaneNormalBack.getRotated(gui->getSlider("Tilt X")->getValue(), ofVec3f(1,0,0));
        basePlaneNormal.rotate(gui->getSlider("Tilt Y")->getValue(), ofVec3f(0,1,0));
        baseplaneUpdated = true;
    } else if (e.target->is("Vertical offset")) {
        basePlaneOffset.z = basePlaneOffsetBack.z + e.value;
        baseplaneUpdated = true;
    } else if (e.target->is("Highest detection\nlevel")){
        maxOffset = e.value;
        ofLogVerbose("GreatSand") << "onSliderEvent(): maxOffset" << maxOffset ;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if(e.target->is("# of averaging slots")){
#if __cplusplus>=201103
        kinectgrabber.numAveragingSlotschannel.send(std::move(e.value));
#else
        kinectgrabber.numAveragingSlotschannel.send(e.value);
#endif
    }
    
    
}

void KinectProjector::saveCalibrationAndSettings(){
    if (kpt.saveCalibration("calibration.xml"))
    {
        ofLogVerbose("GreatSand") << "update(): initialisation: Calibration saved " ;
        saved = true;
    } else {
        ofLogVerbose("GreatSand") << "update(): initialisation: Calibration could not be saved " ;
    }
    if (saveSettings("settings.xml"))
    {
        ofLogVerbose("GreatSand") << "update(): initialisation: Settings saved " ;
    } else {
        ofLogVerbose("GreatSand") << "update(): initialisation: Settings could not be saved " ;
    }
}

bool KinectProjector::loadSettings(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
    xml.setTo("KINECTSETTINGS");
    kinectROI = xml.getValue<ofRectangle>("kinectROI");
    basePlaneNormalBack = xml.getValue<ofVec3f>("basePlaneNormal");
    basePlaneNormal = basePlaneNormalBack;
    basePlaneOffsetBack = xml.getValue<ofVec3f>("basePlaneOffset");
    basePlaneOffset = basePlaneOffsetBack;
    basePlaneEq = xml.getValue<ofVec4f>("basePlaneEq");
    maxOffset = xml.getValue<float>("maxOffset");
    
    return true;
}

bool KinectProjector::saveSettings(string path){
    ofXml xml;
    xml.addChild("KINECTSETTINGS");
    xml.setTo("KINECTSETTINGS");
    xml.addValue("kinectROI", kinectROI);
    xml.addValue("basePlaneNormal", basePlaneNormal);
    xml.addValue("basePlaneOffset", basePlaneOffset);
    xml.addValue("basePlaneEq", basePlaneEq);
    xml.addValue("maxOffset", maxOffset);
    xml.setToParent();
    return xml.save(path);
}







