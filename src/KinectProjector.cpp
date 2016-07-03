//
//  KinectProjector.cpp
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#include "KinectProjector.h"

using namespace states;
using namespace ofxCSG;

//--------------------------------------------------------------
void KinectProjector::setup(int sprojResX, int sprojResY){
	// settings and defaults
	generalState = GENERAL_STATE_CALIBRATION;
    previousGeneralState = GENERAL_STATE_CALIBRATION;
	calibrationState  = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
    previousCalibrationState = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
    ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
    autoCalibState = AUTOCALIB_STATE_INIT_FIRST_PLANE;
    initialisationState = INITIALISATION_STATE_DONE;

    saved = false;
    loaded = false;
    calibrated = false;
    firstImageReady = false;

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
	
    maxOffset = 0;
    maxOffsetSafeRange = 50; // Range above the autocalib measured max offset
    
    // kinectgrabber: start
	kinectgrabber.setup();
    
    // Get projector and kinect width & height
    ofVec2f kinSize = kinectgrabber.getKinectSize();
    kinectResX = kinSize.x;
    kinectResY = kinSize.y;
    projResX = sprojResX;
    projResY = sprojResY;
	kinectROI = ofRectangle(0, 0, kinectResX, kinectResY);
    testPoint.set(kinectResX/2, kinectResY/2);
    
    // Initialize the fbos and images
    FilteredDepthImage.allocate(kinectResX, kinectResY);
    kinectColorImage.allocate(kinectResX, kinectResY);
    thresholdedImage.allocate(kinectResX, kinectResY);
    Dptimg.allocate(20, 20); // Small detailed ROI
    
    //Try to load calibration file if possible
    if (kpt.loadCalibration("calibration.xml"))
    {
        ofLogVerbose("GreatSand") << "setup(): Calibration loaded " ;
        kinectProjMatrix = kpt.getProjectionMatrix();
        ofLogVerbose("GreatSand") << "setup(): kinectProjMatrix: " << kinectProjMatrix ;
        loaded = true;
        calibrated = true;
        generalState = GENERAL_STATE_SANDBOX;
        updateMode();
    } else {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        initialisationState = INITIALISATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        updateMode();
        ofLogVerbose("GreatSand") << "setup(): Calibration could not be loaded, initialisation" ;
    }
    
    //Try to load settings file if possible
    if (loadSettings("settings.xml"))
    {
        ofLogVerbose("GreatSand") << "setup(): Settings loaded " ;
        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
    } else {
        ofLogVerbose("GreatSand") << "setup(): Settings could not be loaded " ;
    }
    
	// finish kinectgrabber setup and start the grabber
    kinectgrabber.setupFramefilter(gradFieldresolution, maxOffset, kinectROI);
    kinectWorldMatrix = kinectgrabber.getWorldMatrix();
    ofLogVerbose("GreatSand") << "setup(): kinectWorldMatrix: " << kinectWorldMatrix ;
    
    // Setup gradient field
    setupGradientField();
    
    clearFbos();

    kinectgrabber.start(); // Start the acquisition
}

//--------------------------------------------------------------
void KinectProjector::setupGradientField(){
    // gradient field config and initialisation
    gradFieldcols = kinectResX / gradFieldresolution;
    gradFieldrows = kinectResY / gradFieldresolution;
    
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
}

//--------------------------------------------------------------
void KinectProjector::updateNativeScale(float scaleMin, float scaleMax){
    FilteredDepthImage.setNativeScale(scaleMin, scaleMax);
}
//--------------------------------------------------------------
void KinectProjector::drawChessboard(int x, int y, int chessboardSize) {
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
            //float ofMap(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp=false)
            int x0 = ofMap(i, 0, chessboardX, 0, chessboardSize);
            int y0 = ofMap(j, 0, chessboardY, 0, chessboardSize);
            if (j>0 && i>0) {
                // Not-normalized (on proj screen)
                currentProjectorPoints.push_back(ofVec2f(xf+x0, yf+y0));
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
}

//--------------------------------------------------------------
void KinectProjector::drawTestingPoint(ofVec2f projectedPoint) {
    // Draw a testing point on the projector window
    float ptSize = ofMap(sin(ofGetFrameNum()*0.1), -1, 1, 3, 40);
    ofSetColor(0, 255, 0);
    // Not-normalized (on proj screen)
    ofDrawCircle(projectedPoint.x, projectedPoint.y, ptSize);
    // Normalized coordinates (between 0 and 1)
    //    ofDrawCircle(
    //             ofMap(projectedPoint.x, 0, 1, 0, fboProjWindow.getWidth()),
    //             ofMap(projectedPoint.y, 0, 1, 0, fboProjWindow.getHeight()),
    //             ptSize);
    ofSetColor(255);
}

//--------------------------------------------------------------
void KinectProjector::drawFlowField()
{
    /*
     Uncomment to draw an x at the centre of the screen
     ofPoint screenCenter = ofPoint(ofGetWidth() / 2,ofGetHeight() / 2);
     ofSetColor(255,0,0,255);
     ofLine(screenCenter.x - 50,screenCenter.y-50,screenCenter.x+50,screenCenter.y+50);
     ofLine(screenCenter.x + 50,screenCenter.y-50,screenCenter.x-50,screenCenter.y+50);
     */
    float maxsize = 0;
    for(int rowPos=0; rowPos< gradFieldrows ; rowPos++)
    {
        for(int colPos=0; colPos< gradFieldcols ; colPos++)
        {
            if (rowPos == gradFieldrows/2 && colPos == gradFieldcols/2){
                
            }
            // Find where to draw the arrow
            ofVec2f t = ofVec2f((colPos*gradFieldresolution) + gradFieldresolution/2, rowPos*gradFieldresolution  + gradFieldresolution/2);
            ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));
            
            // Find arrow dimensions
            ofVec2f v2 = gradField[colPos + (rowPos * gradFieldcols)];
            if (v2.length() > maxsize)
                maxsize = v2.length();
            v2 *= arrowLength;
            
            drawArrow(projectedPoint, v2);
        }
    }
}

//--------------------------------------------------------------
void KinectProjector::drawArrow(ofVec2f projectedPoint, ofVec2f v1)
{
    // Draw an arrow of dimensions v1 on the projector fbo at location projectedPoint
    ofFill();
    ofPushMatrix();
    
    ofTranslate(projectedPoint);
    
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, v1.x, v1.y);
    ofDrawCircle(v1.x, v1.y, 5);
    
    ofPopMatrix();
    
}

//--------------------------------------------------------------
void KinectProjector::addPointPair() {
    // Add point pair based on kinect world coordinates
    ofLogVerbose("GreatSand") << "addPointPair(): Adding point pair in kinect world coordinates" ;
    int nDepthPoints = 0;
    for (int i=0; i<cvPoints.size(); i++) {
        ofVec3f worldPoint = getWorldCoord(cvPoints[i].x, cvPoints[i].y);
        if (worldPoint.z > 0)   nDepthPoints++;
    }
    if (nDepthPoints == (chessboardX-1)*(chessboardY-1)) {
        for (int i=0; i<cvPoints.size(); i++) {
            ofVec3f worldPoint = getWorldCoord(cvPoints[i].x, cvPoints[i].y);
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
    
    // Add point pair base on kinect camera coordinate (x, y in 640x480, z in calibrated units)
    //    ofLogVerbose("GreatSand") << "Adding point pair in kinect camera coordinates" ;
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
    //    ofLogVerbose("GreatSand") << resultMessage ;
    //
}

//--------------------------------------------------------------
ofVec2f KinectProjector::computeTransform(ofVec4f vin) // vin is in kinect world coordinate
{
    ofVec4f screenPos = kinectProjMatrix*vin;
    ofVec2f projectedPoint(screenPos.x/screenPos.z, screenPos.y/screenPos.z);
    return projectedPoint;
}

//--------------------------------------------------------------
ofVec4f KinectProjector::getWorldCoord(float x, float y) // vin is in kinect image coordinate
{
    ofVec4f wc = ofVec2f(x, y);
    wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)y*kinectResX+(int)x];
    wc.w = 1;
    ofVec4f retVal = kinectWorldMatrix*wc*wc.z;
    retVal.w = 1;
    return retVal;
}

//--------------------------------------------------------------
// Find the offset of a plane inside the kinectROI to take as maxoffset (over which the depth values are disregarded
void KinectProjector::findMaxOffset(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    
    ofLogVerbose("GreatSand") << "findMaxOffset(): smallROI: " << smallROI ;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    ofLogVerbose("GreatSand") << "findMaxOffset(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    
    if (sw*sh == 0) {
        ofLogVerbose("GreatSand") << "findMaxOffset(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    
    ofVec4f pt;
    
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("GreatSand") << "findMaxOffset(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = ofVec3f(getWorldCoord(x+sl, y+st));//vertexCcvertexCc;
        }
    }
    
    ofLogVerbose("GreatSand") << "findMaxOffset(): Computing plane from points" ;
    ofVec4f eqoff = plane_from_points(points, sw*sh);
    maxOffset = -eqoff.w-maxOffsetSafeRange;
    
    // Update max Offset
    ofLogVerbose("GreatSand") << "findMaxOffset(): maxOffset" << maxOffset ;
    kinectgrabber.setMaxOffset(maxOffset);
}

//--------------------------------------------------------------
// Autoclibration of the sandbox
void KinectProjector::autoCalib(){
    if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE){
        //        fboProjWindow.begin();
        //        ofBackground(255);
        //        fboProjWindow.end();
        //        updateROIAutoSetup();
        //        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        modaltext = "Kinect/projector calibration: Please flatten the sand surface carefully before starting the calibration.";
        //        resultMessage = "Please flatten the sand surface carefully and press [SPACE]. This surface will give the sea level.";
    } else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT){
        if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
            if (kinectROI.getArea() == 0)
            {
                modaltext = "The sand region has not been defined. Please detect or define manually the sand region.";
                autoCalibState = AUTOCALIB_STATE_DONE;
            } else {
                modaltext = "Kinect/projector calibration: Computing base plane, please wait.";
                computeBasePlane(); // Find base plane
                
                autoCalibPts = new ofPoint[10];
                float cs = 2*chessboardSize/3;
                float css = 3*chessboardSize/4;
                ofPoint sc = ofPoint(projResX/2,projResY/2);
                
                // Prepare 10 locations for the calibration chessboard
                autoCalibPts[0] = ofPoint(cs,cs)-sc;
                autoCalibPts[1] = ofPoint(projResX-cs,cs)-sc;
                autoCalibPts[2] = ofPoint(projResX-cs,projResY-cs)-sc;
                autoCalibPts[3] = ofPoint(cs,projResY-cs)-sc;
                autoCalibPts[4] = ofPoint(projResX/2+cs,projResY/2)-sc;
                autoCalibPts[5] = ofPoint(css,css)-sc;
                autoCalibPts[6] = ofPoint(projResX-css,css)-sc;
                autoCalibPts[7] = ofPoint(projResX-css,projResY-css)-sc;
                autoCalibPts[8] = ofPoint(css,projResY-css)-sc;
                autoCalibPts[9] = ofPoint(projResX/2-cs,projResY/2)-sc;
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
                    ofPoint dispPt = ofPoint(projResX/2,projResY/2)+autoCalibPts[currentCalibPts]; // Compute next chessboard position
                    drawChessboard(dispPt.x, dispPt.y, chessboardSize); // We can now draw the next chess board
                } else {
                    // Wecannot find the chessboard
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
                findMaxOffset(); // Find max offset
                autoCalibState = AUTOCALIB_STATE_COMPUTE;
            } else { // We ask for higher points
                //                resultMessage = "Please put a board on the sandbox and press [SPACE]";
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
        }
        autoCalibState = AUTOCALIB_STATE_DONE;
    } else if (autoCalibState == AUTOCALIB_STATE_DONE){
    }
}

//--------------------------------------------------------------
// Find kinect ROI of the sandbox
void KinectProjector::updateROIAutoSetup(){
    //updateROIFromColorImage();
    updateROIFromDepthImage();
}

//--------------------------------------------------------------
// Find kinect ROI of the sandbox from the colored image
void KinectProjector::updateROIFromColorImage(){
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) { // set kinect to max depth range
        ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
        large = ofPolyline();
        threshold = 90;
        
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
        while (threshold < 255){
            //            ofLogVerbose("GreatSand") << "Increasing threshold : " << threshold ;
            //                            thresholdedImage.mirror(verticalMirror, horizontalMirror);
            //        CV_THRESH_BINARY      =0,  /* value = value > threshold ? max_value : 0       */
            //        CV_THRESH_BINARY_INV  =1,  /* value = value > threshold ? 0 : max_value       */
            //        CV_THRESH_TRUNC       =2,  /* value = value > threshold ? threshold : value   */
            //        CV_THRESH_TOZERO      =3,  /* value = value > threshold ? value : 0           */
            //        CV_THRESH_TOZERO_INV  =4,  /* value = value > threshold ? 0 : value           */
            //        CV_THRESH_MASK        =7,
            //        CV_THRESH_OTSU        =8  /* use Otsu algorithm to choose the optimal threshold value;
            //                                   combine the flag with one of the above CV_THRESH_* values */
            kinectColorImage.setROI(0, 0, kinectResX, kinectResY);
            thresholdedImage = kinectColorImage;
            //        cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold+50, 255, CV_THRESH_TOZERO_INV);
            
            // Threshold the color image of the kinect
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_BINARY_INV);
            
            // Find contours
            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true);
            
            ofPolyline small = ofPolyline();
            for (int i = 0; i < contourFinder.nBlobs; i++) {
                ofxCvBlob blobContour = contourFinder.blobs[i];
                if (blobContour.hole) {
                    ofPolyline poly = ofPolyline(blobContour.pts);//.getResampledByCount(50);
                    // We keep only the contours surrounding at least the center of the screen
                    if (poly.inside(kinectResX/2, kinectResY/2))
                    {
                        //                        ofLogVerbose("GreatSand") << "We found a contour lines surroundings the center of the screen" ;
                        if (small.size() == 0 || poly.getArea() < small.getArea()) {
                            //                            ofLogVerbose("GreatSand") << "We take the smallest contour line surroundings the center of the screen at a given threshold level" ;
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
        //        if (threshold > 255) {
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        ofLogVerbose("GreatSand") << "updateROIFromColorImage(): kinectROI : " << kinectROI ;
        // We are finished, set back kinect depth range and update ROI
        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
        updateKinectGrabberROI();
        //update the mesh
        setupMesh();
        //        }
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
    }
}

//--------------------------------------------------------------
// Find kinect ROI of the sandbox from the depth image
void KinectProjector::updateROIFromDepthImage(){
    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) {
        modaltext = "Detecting sand area: Please wait...";
        // set kinect to max depth range
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_INIT: Set maximal ROI for kinect" ;
        kinectROI = ofRectangle(0, 0, kinectResX, kinectResY);
        updateKinectGrabberROI();
        
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_INIT: Wait for kinectgrabber to reset buffers" ;
        while (kinectgrabber.isFirstImageReady()){
        } // Wait for kinectgrabber to reset buffers
        
        firstImageReady = false; // Now we waite for a clean new depth frame
        ROICalibrationState = ROI_CALIBRATION_STATE_READY_TO_MOVE_UP;
        
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_INIT: Wait for clean new frame" ;
        
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_READY_TO_MOVE_UP) {
        if (firstImageReady)
        {
            ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_READY_TO_MOVE_UP: got a stable depth image" ;
            ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
            large = ofPolyline();
            
            //            // Find max and min of the filtered depth iamge
            //            float maxval = FilteredDepthImage.getFloatPixelsRef().getData()[0];
            //            float minval = FilteredDepthImage.getFloatPixelsRef().getData()[0];
            //            float xf;
            //            for (int i = 0; i<640*480; i ++){
            //                xf = FilteredDepthImage.getFloatPixelsRef().getData()[i];
            //                if (xf > maxval)
            //                    maxval = xf;
            //                if (xf < minval)
            //                    minval = xf;
            //            }
            //            if (maxval > 1500)
            //                maxval = 1500;
            //            if (minval < 500)
            //                minval = 500;
            //            ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): FilteredDepthImage maxval : " << maxval << " thresholdedImage minval : " << minval ;
            //
            //            // Compute a scaled grayscale image
            //            for (int i = 0; i<640*480; i ++){
            //                thresholdedImage.getPixels().getData()[i] = (int)((FilteredDepthImage.getFloatPixelsRef().getData()[i]-minval)/(maxval-minval)*255.0);
            //            }
            //
            //            //Check values for debug
            //            maxval = thresholdedImage.getPixels().getData()[0];
            //            minval = thresholdedImage.getPixels().getData()[0];
            //            for (int i = 0; i<640*480; i ++){
            //                xf = thresholdedImage.getPixels().getData()[i];
            //                if (xf > maxval)
            //                    maxval = xf;
            //                if (xf < minval)
            //                    minval = xf;
            //            }
            //            ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): thresholdedImage maxval : " << maxval << " temp minval : " << minval ;
            ofxCvFloatImage temp;
            temp.setFromPixels(FilteredDepthImage.getFloatPixelsRef().getData(), kinectResX, kinectResY);
            temp.setNativeScale(FilteredDepthImage.getNativeScaleMin(), FilteredDepthImage.getNativeScaleMax());
            temp.convertToRange(0, 1);
            
            //Check values for debug
            float maxval = -1000.0;
            float minval = 1000.0;
            float xf;
            for (int i = 0; i<640*480; i ++){
                xf = temp.getFloatPixelsRef().getData()[i];
                
                if (xf > maxval)
                    maxval = xf;
                if (xf < minval)
                    minval = xf;
            }
            cout << "temp maxval : " << maxval << " temp minval : " << minval << endl;
            
            thresholdedImage.setFromPixels(temp.getFloatPixelsRef());//kinectColorImage.getPixels());
            
            //Check values for debug
            maxval = -1000.0;
            minval = 1000.0;
            for (int i = 0; i<640*480; i ++){
                xf = thresholdedImage.getPixels().getData()[i];
                
                if (xf > maxval)
                    maxval = xf;
                if (xf < minval)
                    minval = xf;
            }
            cout << "thresholdedImage maxval : " << maxval << " thresholdedImage minval : " << minval << endl;
        }
        threshold = 0; // We go from the higher distance to the kinect (lower position) to the lower distance
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
        //        if (threshold < 255){
        while (threshold < 255){
            //                            thresholdedImage.mirror(verticalMirror, horizontalMirror);
            //cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), highThresh+10, 255, CV_THRESH_TOZERO_INV);
            //        CV_THRESH_BINARY      =0,  /* value = value > threshold ? max_value : 0       */
            //        CV_THRESH_BINARY_INV  =1,  /* value = value > threshold ? 0 : max_value       */
            //        CV_THRESH_TRUNC       =2,  /* value = value > threshold ? threshold : value   */
            //        CV_THRESH_TOZERO      =3,  /* value = value > threshold ? value : 0           */
            //        CV_THRESH_TOZERO_INV  =4,  /* value = value > threshold ? 0 : value           */
            //        CV_THRESH_MASK        =7,
            //        CV_THRESH_OTSU        =8  /* use Otsu algorithm to choose the optimal threshold value;
            //                                   combine the flag with one of the above CV_THRESH_* values */
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), 255-threshold, 255, CV_THRESH_TOZERO_INV);//CV_THRESH_TOZERO);
            thresholdedImage.updateTexture();
            
            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true, false);
            //contourFinder.findContours(thresholdedImage);
            //ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
            
            ofPolyline small = ofPolyline();
            for (int i = 0; i < contourFinder.nBlobs; i++) {
                ofxCvBlob blobContour = contourFinder.blobs[i];
                if (blobContour.hole) {
                    ofPolyline poly = ofPolyline(blobContour.pts);//.getResampledByCount(50);
                    
                    if (poly.inside(kinectResX/2, kinectResY/2))
                    {
                        //                        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): We found a contour lines surroundings the center of the screen" ;
                        if (small.size() == 0 || poly.getArea() < small.getArea()) {
                            //                            ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): We take the smallest contour line surroundings the center of the screen at a given threshold level" ;
                            small = poly;
                        }
                    }
                }
            }
            if (large.getArea() < small.getArea())
            {
                //We take the largest contour line surroundings the center of the screen at all threshold level
                ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): updating ROI" ;
                large = small;
            }
            threshold+=1;
        }// else {
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        ofLogVerbose("GreatSand") << "updateROIFromDepthImage(): final kinectROI : " << kinectROI ;
        // We are finished, set back kinect depth range and update ROI
        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
        updateKinectGrabberROI();
        modaltext = "Detecting sand area: Sand area successfully detected.";
        //  }
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
    }
}

//--------------------------------------------------------------
void KinectProjector::updateROIManualSetup(){
    //Manually setup the ROI
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    
    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) {
        resultMessage = "Please click on first ROI corner";
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
        resultMessage = "Please click on second ROI corner";
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
        resultMessage = "Manual ROI update done";
    }
}

//--------------------------------------------------------------
void KinectProjector::updateKinectGrabberROI(){
    // Send the new mode informations to the kinect grabber
#if __cplusplus>=201103
    kinectgrabber.ROIchannel.send(std::move(kinectROI));
#else
    kinectgrabber.ROIchannel.send(kinectROI);
#endif
}
//--------------------------------------------------------------
void KinectProjector::updateMode(){
    if (generalState == GENERAL_STATE_CALIBRATION)
        // White background on the projector for calibration
        ofBackground(255);
    else
        // Black background on the projector for playing with the sandbox
        ofBackground(0);
    
    ofLogVerbose("GreatSand") << "updateMode(): General state: " << generalState ;
    ofLogVerbose("GreatSand") << "updateMode(): Calibration state: " << calibrationState ;
    
    // Send the new mode informations to the kinect grabber
#if __cplusplus>=201103
    kinectgrabber.generalStateChannel.send(std::move(generalState));
    kinectgrabber.calibrationStateChannel.send(std::move(calibrationState));
#else
    kinectgrabber.generalStateChannel.send(generalState);
    kinectgrabber.calibrationStateChannel.send(calibrationState);
#endif
}

//--------------------------------------------------------------
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

//--------------------------------------------------------------
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







