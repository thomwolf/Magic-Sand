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

#include "KinectProjector.h"

using namespace ofxCSG;

KinectProjector::KinectProjector(std::shared_ptr<ofAppBaseWindow> const& p)
:ROIcalibrated(false),
projKinectCalibrated(false),
calibrating (false),
basePlaneUpdated (false),
projKinectCalibrationUpdated (false),
ROIUpdated (false),
imageStabilized (false),
waitingForFlattenSand (false),
drawKinectView(false)
{
    projWindow = p;
}

void KinectProjector::setup(bool sdisplayGui){
	ofAddListener(ofEvents().exit, this, &KinectProjector::exit);
	
	// instantiate the modal windows //
    modalTheme = make_shared<ofxModalThemeProjKinect>();
    confirmModal = make_shared<ofxModalConfirm>();
    confirmModal->setTheme(modalTheme);
    confirmModal->addListener(this, &KinectProjector::onConfirmModalEvent);
    confirmModal->setButtonLabel("Ok");
    
    calibModal = make_shared<ofxModalAlert>();
    calibModal->setTheme(modalTheme);
    calibModal->addListener(this, &KinectProjector::onCalibModalEvent);
    calibModal->setButtonLabel("Cancel");
        
	displayGui = sdisplayGui;

    //Check the size and location of the second window to fit the second screen
    //secondScreenFound = checkProjectorWindow();
    //if (!secondScreenFound){
    //    confirmModal->setMessage("Projector not found. Please check that the projector is (1) connected, (2) powerer and (3) not in mirror mode.");
    //    confirmModal->show();
    //}
    
    // calibration chessboard config
	chessboardSize = 300;
	chessboardX = 5;
    chessboardY = 4;
    
    // 	Gradient Field
	gradFieldResolution = 10;
    arrowLength = 25;
	
    // Setup default base plane
	basePlaneNormalBack = ofVec3f(0,0,1); // This is our default baseplane normal
	basePlaneOffsetBack= ofVec3f(0,0,870); // This is our default baseplane offset
    basePlaneNormal = basePlaneNormalBack;
    basePlaneOffset = basePlaneOffsetBack;
    basePlaneEq=getPlaneEquation(basePlaneOffset,basePlaneNormal);
    maxOffsetBack = basePlaneOffset.z-300;
    maxOffset = maxOffsetBack;
    maxOffsetSafeRange = 50; // Range above the autocalib measured max offset

    // kinectgrabber: start & default setup
	kinectOpened = kinectgrabber.setup();
	if (!kinectOpened){
	    confirmModal->setMessage("Cannot connect to Kinect. Please check that the kinect is (1) connected, (2) powerer and (3) not used by another application.");
	    confirmModal->show();
	}
	spatialFiltering = true;
    followBigChanges = false;
    numAveragingSlots = 15;
    
    // Get projector and kinect width & height
    projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
    kinectRes = kinectgrabber.getKinectSize();
	kinectROI = ofRectangle(0, 0, kinectRes.x, kinectRes.y);
    
    // Initialize the fbos and images
    FilteredDepthImage.allocate(kinectRes.x, kinectRes.y);
    kinectColorImage.allocate(kinectRes.x, kinectRes.y);
    thresholdedImage.allocate(kinectRes.x, kinectRes.y);
    Dptimg.allocate(20, 20); // Small detailed ROI
    
	kpt = new ofxKinectProjectorToolkit(projRes, kinectRes);

	//Try to load calibration file if possible
    if (kpt->loadCalibration("settings/calibration.xml"))
    {
        ofLogVerbose("KinectProjector") << "KinectProjector.setup(): Calibration loaded " ;
        kinectProjMatrix = kpt->getProjectionMatrix();
        ofLogVerbose("KinectProjector") << "KinectProjector.setup(): kinectProjMatrix: " << kinectProjMatrix ;
        projKinectCalibrated = true;
    } else {
        if (displayGui){
            // Show auto calibration modal window
            confirmModal->setMessage("No calibration file could be found for the kinect and the projector. Starting calibration process.");
            confirmModal->show();
        }
        ofLogVerbose("KinectProjector") << "KinectProjector.setup(): Calibration could not be loaded" ;
    }
    
    //Try to load settings file if possible
    if (loadSettings())
    {
        ofLogVerbose("KinectProjector") << "KinectProjector.setup(): Settings loaded " ;
        ROIcalibrated = true;
    } else {
        ofLogVerbose("KinectProjector") << "KinectProjector.setup(): Settings could not be loaded " ;
    }
    
	// finish kinectgrabber setup and start the grabber
    kinectgrabber.setupFramefilter(gradFieldResolution, maxOffset, kinectROI, spatialFiltering, followBigChanges, numAveragingSlots);
    kinectWorldMatrix = kinectgrabber.getWorldMatrix();
    ofLogVerbose("KinectProjector") << "KinectProjector.setup(): kinectWorldMatrix: " << kinectWorldMatrix ;
    
    // Setup gradient field
    setupGradientField();
    
    fboProjWindow.allocate(projRes.x, projRes.y, GL_RGBA);
    fboProjWindow.begin();
    ofClear(255, 255, 255, 0);
    fboProjWindow.end();
    
    fboMainWindow.allocate(kinectRes.x, kinectRes.y, GL_RGBA);
    fboMainWindow.begin();
    ofClear(255, 255, 255, 0);
    fboMainWindow.end();
    
    if (displayGui)
        setupGui();
    
    kinectgrabber.start(); // Start the acquisition
}

void KinectProjector::exit(ofEventArgs& e){
    if (saveSettings())
    {
        ofLogVerbose("KinectProjector") << "exit(): Settings saved " ;
    } else {
        ofLogVerbose("KinectProjector") << "exit(): Settings could not be saved " ;
    }
}

void KinectProjector::setupGradientField(){
    gradFieldcols = kinectRes.x / gradFieldResolution;
    gradFieldrows = kinectRes.y / gradFieldResolution;
    
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
}

void KinectProjector::setGradFieldResolution(int sgradFieldResolution){
    gradFieldResolution = sgradFieldResolution;
    setupGradientField();
    kinectgrabber.performInThread([sgradFieldResolution](KinectGrabber & kg) {
        kg.setGradFieldResolution(sgradFieldResolution);
    });
}

void KinectProjector::update(){
    // Clear updated state variables
    basePlaneUpdated = false;
    ROIUpdated = false;
    projKinectCalibrationUpdated = false;

	if (displayGui)
		gui->update();

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
        
        // Get gradient field from kinect grabber
        kinectgrabber.gradient.tryReceive(gradField);
        
        // Update grabber stored frame number
        kinectgrabber.lock();
        kinectgrabber.decStoredframes();
        kinectgrabber.unlock();
        
        // Is the depth image stabilized
        imageStabilized = kinectgrabber.isImageStabilized();
        
        // Are we calibrating ?
        if (calibrating && !waitingForFlattenSand) {
            updateCalibration();
        } else {
			//ofEnableAlphaBlending();
			fboMainWindow.begin();
            if (drawKinectView){
                FilteredDepthImage.draw(0, 0);
				ofNoFill();
				ofDrawRectangle(kinectROI);
				ofDrawRectangle(0, 0, kinectRes.x, kinectRes.y);
			} else {
                ofClear(0, 0, 0, 0);
            }
            fboMainWindow.end();
        }
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

void KinectProjector::updateROIFromCalibration(){
	ofVec2f a = worldCoordTokinectCoord(projCoordAndWorldZToWorldCoord(0, 0, basePlaneOffset.z));
	ofVec2f b = worldCoordTokinectCoord(projCoordAndWorldZToWorldCoord(projRes.x, 0, basePlaneOffset.z));
	ofVec2f c = worldCoordTokinectCoord(projCoordAndWorldZToWorldCoord(projRes.x, projRes.y, basePlaneOffset.z));
	ofVec2f d = worldCoordTokinectCoord(projCoordAndWorldZToWorldCoord(0, projRes.y, basePlaneOffset.z));
	float x1 = max(a.x, d.x);
	float x2 = min(b.x, c.x);
	float y1 = max(a.y, b.y);
	float y2 = min(c.y, d.y);
	ofRectangle smallKinectROI = ofRectangle(ofPoint(max(x1, kinectROI.getLeft()), max(y1, kinectROI.getTop())), ofPoint(min(x2, kinectROI.getRight()), min(y2, kinectROI.getBottom())));
	kinectROI = smallKinectROI;

	kinectROI.standardize();
	ofLogVerbose("KinectProjector") << "updateROIFromCalibration(): final kinectROI : " << kinectROI;
	setNewKinectROI();
}

//TODO: update color image ROI acquisition to use calibration modal
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
            contourFinder.findContours(thresholdedImage, 12, kinectRes.x*kinectRes.y, 5, true);
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
            ofLogVerbose("KinectProjector") << "KinectProjector.updateROIFromColorImage(): small.getArea(): " << small.getArea() ;
            ofLogVerbose("KinectProjector") << "KinectProjector.updateROIFromColorImage(): large.getArea(): " << large.getArea() ;
            if (large.getArea() < small.getArea())
            {
                ofLogVerbose("KinectProjector") << "updateROIFromColorImage(): We take the largest contour line surroundings the center of the screen at all threshold level" ;
                large = small;
            }
            threshold+=1;
        }
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        ofLogVerbose("KinectProjector") << "updateROIFromColorImage(): kinectROI : " << kinectROI ;
        ROICalibState = ROI_CALIBRATION_STATE_DONE;
        setNewKinectROI();
    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
    }
}

void KinectProjector::updateROIFromDepthImage(){
    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
        calibModal->setMessage("Enlarging acquisition area & resetting buffers.");
        setMaxKinectGrabberROI();
        calibModal->setMessage("Stabilizing acquisition.");
        ROICalibState = ROI_CALIBRATION_STATE_READY_TO_MOVE_UP;
    } else if (ROICalibState == ROI_CALIBRATION_STATE_READY_TO_MOVE_UP && imageStabilized) {
        calibModal->setMessage("Scanning depth field to find sandbox walls.");
        ofLogVerbose("KinectProjector") << "updateROIFromDepthImage(): ROI_CALIBRATION_STATE_READY_TO_MOVE_UP: got a stable depth image" ;
        ROICalibState = ROI_CALIBRATION_STATE_MOVE_UP;
        large = ofPolyline();
        ofxCvFloatImage temp;
        temp.setFromPixels(FilteredDepthImage.getFloatPixelsRef().getData(), kinectRes.x, kinectRes.y);
        temp.setNativeScale(FilteredDepthImage.getNativeScaleMin(), FilteredDepthImage.getNativeScaleMax());
        temp.convertToRange(0, 1);
        thresholdedImage.setFromPixels(temp.getFloatPixelsRef());
        threshold = 0; // We go from the higher distance to the kinect (lower position) to the lower distance
    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
        while (threshold < 255){
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), 255-threshold, 255, CV_THRESH_TOZERO_INV);
            thresholdedImage.updateTexture();
            contourFinder.findContours(thresholdedImage, 12, kinectRes.x*kinectRes.y, 5, true, false);
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
                ofLogVerbose("KinectProjector") << "updateROIFromDepthImage(): updating ROI" ;
                large = small;
            }
            threshold+=1;
        }
        if (large.getArea() == 0)
        {
            calibModal->hide();
            confirmModal->setTitle("Calibration failed");
            confirmModal->setMessage("The sandbox walls could not be found.");
            confirmModal->show();
            calibrating = false;
        } else {
            kinectROI = large.getBoundingBox();
//            insideROIPoly = large.getResampledBySpacing(10);
            kinectROI.standardize();
            calibModal->setMessage("Sand area successfully detected");
            ofLogVerbose("KinectProjector") << "updateROIFromDepthImage(): final kinectROI : " << kinectROI ;
            setNewKinectROI();
            if (calibrationState == CALIBRATION_STATE_ROI_AUTO_DETERMINATION){
                calibrating = false;
                calibModal->hide();
            }
        }
        ROICalibState = ROI_CALIBRATION_STATE_DONE;
    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
    }
}
//TODO: Add manual ROI calibration
void KinectProjector::updateROIManualCalibration(){
//    fboProjWindow.begin();
//    ofBackground(255);
//    fboProjWindow.end();
//    
//    if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
//        kinectROIManualCalib.setSize(ofGetMouseX()-kinectROIManualCalib.x,ofGetMouseY()-kinectROIManualCalib.y);
//    }
//    
//    if (ROICalibState == ROI_CALIBRATION_STATE_INIT) {
//        resultMessage = "Please click on first ROI corner";
//    } else if (ROICalibState == ROI_CALIBRATION_STATE_MOVE_UP) {
//        resultMessage = "Please click on second ROI corner";
//    } else if (ROICalibState == ROI_CALIBRATION_STATE_DONE){
//        resultMessage = "Manual ROI update done";
//    }
}

void KinectProjector::setMaxKinectGrabberROI(){
    updateKinectGrabberROI(ofRectangle(0, 0, kinectRes.x, kinectRes.y));
}

void KinectProjector::setNewKinectROI(){
    // Cast to integer values
    kinectROI.x = static_cast<int>(kinectROI.x);
    kinectROI.y = static_cast<int>(kinectROI.y);
    kinectROI.width = static_cast<int>(kinectROI.width);
    kinectROI.height = static_cast<int>(kinectROI.height);
    
    // Update states variables
    ROIcalibrated = true;
    ROIUpdated = true;
    saveCalibrationAndSettings();
    updateKinectGrabberROI(kinectROI);
}

void KinectProjector::updateKinectGrabberROI(ofRectangle ROI){
    kinectgrabber.performInThread([ROI](KinectGrabber & kg) {
        kg.setKinectROI(ROI);
    });
//    while (kinectgrabber.isImageStabilized()){
//    } // Wait for kinectgrabber to reset buffers
    imageStabilized = false; // Now we can wait for a clean new depth frame
}

void KinectProjector::updateProjKinectAutoCalibration(){
    if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE){
        if (!ROIcalibrated){
            updateROIAutoCalibration();
        } else {
            calibModal->setMessage("Enlarging acquisition area & resetting buffers.");
            setMaxKinectGrabberROI();
            kinectgrabber.performInThread([](KinectGrabber & kg) {
                kg.setMaxOffset(0);
            });
            calibModal->setMessage("Stabilizing acquisition.");
            autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        }
    } else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT && imageStabilized){
        calibModal->setMessage("Acquiring sea level plane.");
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
    } else if (autoCalibState == AUTOCALIB_STATE_NEXT_POINT && imageStabilized){
        if (currentCalibPts < 5 || (upframe && currentCalibPts < 10)) {
            if (!upframe){
                string mess = "Acquiring low level calibration point "+std::to_string(currentCalibPts+1)+"/5.";
                calibModal->setMessage(mess);
            } else {
                string mess = "Acquiring high level calibration point "+std::to_string(currentCalibPts-4)+"/5.";
                calibModal->setMessage(mess);
            }
            
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
                    kinectColorImage.updateTexture();
                    fboMainWindow.begin();
                    kinectColorImage.draw(0,0);
                    fboMainWindow.end();
                    
                    ofLogVerbose("KinectProjector") << "autoCalib(): Chessboard found for point :" << currentCalibPts ;
                    bool okchess = addPointPair();
                    
                    if (okchess) {
                        fboProjWindow.begin(); // Clear projector
                        ofBackground(255);
                        fboProjWindow.end();
                        cleared = false;
                        trials = 0;
                        currentCalibPts++;
                    } else {
                        // We cannot get all depth points for the chessboard
                        trials++;
                        ofLogVerbose("KinectProjector") << "autoCalib(): Depth points of chessboard not allfound on trial : " << trials ;
                        if (trials >10) {
                            // Move the chessboard closer to the center of the screen
                            ofLogVerbose("KinectProjector") << "autoCalib(): Chessboard could not be found moving chessboard closer to center " ;
                            autoCalibPts[currentCalibPts] = 4*autoCalibPts[currentCalibPts]/5;
                            fboProjWindow.begin(); // Clear projector
                            ofBackground(255);
                            fboProjWindow.end();
                            cleared = false;
                            trials = 0;
                        }
                    }
                }
            } else {
                if (cleared == false) {
                    ofLogVerbose("KinectProjector") << "autoCalib(): Clear screen found, drawing next chessboard" ;
                    cleared = true; // The cleared fbo screen was seen by the kinect
                    ofPoint dispPt = ofPoint(projRes.x/2,projRes.y/2)+autoCalibPts[currentCalibPts]; // Compute next chessboard position
                    drawChessboard(dispPt.x, dispPt.y, chessboardSize); // We can now draw the next chess board
                } else {
                    // We cannot find the chessboard
                    trials++;
                    ofLogVerbose("KinectProjector") << "autoCalib(): Chessboard not found on trial : " << trials ;
                    if (trials >10) {
                        // Move the chessboard closer to the center of the screen
                        ofLogVerbose("KinectProjector") << "autoCalib(): Chessboard could not be found moving chessboard closer to center " ;
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
                calibModal->setMessage("Updating acquision ceiling.");
                updateMaxOffset(); // Find max offset
                autoCalibState = AUTOCALIB_STATE_COMPUTE;
            } else { // We ask for higher points
                calibModal->hide();
                confirmModal->show();
                confirmModal->setMessage("Please cover the sandbox with a board and press ok.");
            }
        }
    } else if (autoCalibState == AUTOCALIB_STATE_COMPUTE){
        updateKinectGrabberROI(kinectROI); // Goes back to kinectROI and maxoffset
        kinectgrabber.performInThread([this](KinectGrabber & kg) {
            kg.setMaxOffset(this->maxOffset);
        });
        if (pairsKinect.size() == 0) {
            ofLogVerbose("KinectProjector") << "autoCalib(): Error: No points acquired !!" ;
            calibModal->hide();
            confirmModal->setTitle("Calibration failed");
            confirmModal->setMessage("No point could be acquired. ");
            confirmModal->show();
            calibrating = false;
        } else {
            ofLogVerbose("KinectProjector") << "autoCalib(): Calibrating" ;
            kpt->calibrate(pairsKinect, pairsProjector);
            kinectProjMatrix = kpt->getProjectionMatrix();

			updateROIFromCalibration(); // Compute the limite of the ROI according to the projected area 

            projKinectCalibrated = true; // Update states variables
            projKinectCalibrationUpdated = true;
            calibrating = false;
            calibModal->setMessage("Calibration successfull.");
            calibModal->hide();
            //saveCalibrationAndSettings(); // Already done in updateROIFromCalibration
        }
        autoCalibState = AUTOCALIB_STATE_DONE;
    } else if (autoCalibState == AUTOCALIB_STATE_DONE){
    }
}
//TODO: Add manual Prj Kinect calibration
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
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    ofLogVerbose("KinectProjector") << "updateBasePlane(): smallROI: " << smallROI ;
    int sw = static_cast<int>(smallROI.width);
    int sh = static_cast<int>(smallROI.height);
    int sl = static_cast<int>(smallROI.getLeft());
    int st = static_cast<int>(smallROI.getTop());
    ofLogVerbose("KinectProjector") << "updateBasePlane(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    if (sw*sh == 0) {
        ofLogVerbose("KinectProjector") << "updateBasePlane(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    ofVec4f pt;
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("KinectProjector") << "updateBasePlane(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = kinectCoordToWorldCoord(x+sl, y+st);
        }
    }
    ofLogVerbose("KinectProjector") << "updateBasePlane(): Computing plane from points" ;
    basePlaneEq = plane_from_points(points, sw*sh);
    basePlaneNormal = ofVec3f(basePlaneEq);
    basePlaneOffset = ofVec3f(0,0,-basePlaneEq.w);
    basePlaneNormalBack = basePlaneNormal;
    basePlaneOffsetBack = basePlaneOffset;
    basePlaneUpdated = true;
}

void KinectProjector::updateMaxOffset(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    ofLogVerbose("KinectProjector") << "updateMaxOffset(): smallROI: " << smallROI ;
    int sw = static_cast<int>(smallROI.width);
    int sh = static_cast<int>(smallROI.height);
    int sl = static_cast<int>(smallROI.getLeft());
    int st = static_cast<int>(smallROI.getTop());
    ofLogVerbose("KinectProjector") << "updateMaxOffset(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    if (sw*sh == 0) {
        ofLogVerbose("KinectProjector") << "updateMaxOffset(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    ofVec4f pt;
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("KinectProjector") << "updateMaxOffset(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = kinectCoordToWorldCoord(x+sl, y+st);//vertexCcvertexCc;
        }
    }
    ofLogVerbose("KinectProjector") << "updateMaxOffset(): Computing plane from points" ;
    ofVec4f eqoff = plane_from_points(points, sw*sh);
    maxOffset = -eqoff.w-maxOffsetSafeRange;
    maxOffsetBack = maxOffset;
    // Update max Offset
    ofLogVerbose("KinectProjector") << "updateMaxOffset(): maxOffset" << maxOffset ;
    kinectgrabber.performInThread([this](KinectGrabber & kg) {
        kg.setMaxOffset(this->maxOffset);
    });
}

bool KinectProjector::addPointPair() {
    bool okchess = true;
    string resultMessage;
    ofLogVerbose("KinectProjector") << "addPointPair(): Adding point pair in kinect world coordinates" ;
    int nDepthPoints = 0;
    for (int i=0; i<cvPoints.size(); i++) {
        ofVec3f worldPoint = kinectCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
        if (worldPoint.z > 0)   nDepthPoints++;
    }
    if (nDepthPoints == (chessboardX-1)*(chessboardY-1)) {
        for (int i=0; i<cvPoints.size(); i++) {
            ofVec3f worldPoint = kinectCoordToWorldCoord(cvPoints[i].x, cvPoints[i].y);
//            cout << "Kinect: " << worldPoint << "Proj: " << currentProjectorPoints[i] << endl;
            pairsKinect.push_back(worldPoint);
            pairsProjector.push_back(currentProjectorPoints[i]);
        }
        resultMessage = "addPointPair(): Added " + ofToString((chessboardX-1)*(chessboardY-1)) + " points pairs.";
    } else {
        resultMessage = "addPointPair(): Points not added because not all chessboard\npoints' depth known. Try re-positionining.";
        okchess = false;
    }
    ofLogVerbose("KinectProjector") << resultMessage ;
    return okchess;
}

void KinectProjector::askToFlattenSand(){
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    confirmModal->setMessage("Please flatten the sand surface.");
    confirmModal->show();
    waitingForFlattenSand = true;
}

void KinectProjector::drawProjectorWindow(){
    fboProjWindow.draw(0,0);
}

void KinectProjector::drawMainWindow(float x, float y, float width, float height){
	fboMainWindow.draw(x,y, width, height);
	if (displayGui)
		gui->draw();
}

void KinectProjector::drawChessboard(int x, int y, int chessboardSize) {
    fboProjWindow.begin();
    ofFill();
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

void KinectProjector::drawGradField()
{
    ofClear(255, 0);
    for(int rowPos=0; rowPos< gradFieldrows ; rowPos++)
    {
        for(int colPos=0; colPos< gradFieldcols ; colPos++)
        {
            float x = colPos*gradFieldResolution + gradFieldResolution/2;
            float y = rowPos*gradFieldResolution  + gradFieldResolution/2;
            ofVec2f projectedPoint = kinectCoordToProjCoord(x, y);
            int ind = colPos + rowPos * gradFieldcols;
            ofVec2f v2 = gradField[ind];
            v2 *= arrowLength;

            ofSetColor(255,0,0,255);
            if (ind == fishInd)
                ofSetColor(0,255,0,255);
            
            drawArrow(projectedPoint, v2);
        }
    }
}

void KinectProjector::drawArrow(ofVec2f projectedPoint, ofVec2f v1)
{
    float angle = ofRadToDeg(atan2(v1.y,v1.x));
    float length = v1.length();
    ofFill();
    ofPushMatrix();
    ofTranslate(projectedPoint);
    ofRotate(angle);
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, length, 0);
    ofDrawLine(length, 0, length-7, 5);
    ofDrawLine(length, 0, length-7, -5);
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

ofVec3f KinectProjector::projCoordAndWorldZToWorldCoord(float projX, float projY, float worldZ)
{
	float a = kinectProjMatrix(0, 0) - kinectProjMatrix(2, 0)*projX;
	float b = kinectProjMatrix(0, 1) - kinectProjMatrix(2, 1)*projX;
	float c = (kinectProjMatrix(2, 2)*worldZ + 1)*projX - (kinectProjMatrix(0, 2)*worldZ + kinectProjMatrix(0, 3));
	float d = kinectProjMatrix(1, 0) - kinectProjMatrix(2, 0)*projY;
	float e = kinectProjMatrix(1, 1) - kinectProjMatrix(2, 1)*projY;
	float f = (kinectProjMatrix(2, 2)*worldZ + 1)*projY - (kinectProjMatrix(1, 2)*worldZ + kinectProjMatrix(1, 3));
	
	float det = a*e - b*d;
	if (det == 0)
		return ofVec3f(0);
	float y = (a*f - d*c) / det;
	float x = (c*e - b*f) / det;
	return ofVec3f(x, y, worldZ);
}

ofVec3f KinectProjector::kinectCoordToWorldCoord(float x, float y) // x, y in kinect pixel coord
{
    ofVec4f kc = ofVec2f(x, y);
    int ind = static_cast<int>(y) * kinectRes.x + static_cast<int>(x);
    kc.z = FilteredDepthImage.getFloatPixelsRef().getData()[ind];
    kc.w = 1;
    ofVec4f wc = kinectWorldMatrix*kc*kc.z;
    return ofVec3f(wc);
}

ofVec2f KinectProjector::worldCoordTokinectCoord(ofVec3f wc)
{
	float x = (wc.x / wc.z - kinectWorldMatrix(0, 3)) / kinectWorldMatrix(0, 0);
	float y = (wc.y / wc.z - kinectWorldMatrix(1, 3)) / kinectWorldMatrix(1, 1);
	return ofVec2f(x, y);
}

ofVec3f KinectProjector::RawKinectCoordToWorldCoord(float x, float y) // x, y in kinect pixel coord
{
    ofVec4f kc = ofVec3f(x, y, kinectgrabber.getRawDepthAt(static_cast<int>(x), static_cast<int>(y)));
    kc.w = 1;
    ofVec4f wc = kinectWorldMatrix*kc*kc.z;
    return ofVec3f(wc);
}

float KinectProjector::elevationAtKinectCoord(float x, float y) // x, y in kinect pixel coordinate
{
    ofVec4f wc = kinectCoordToWorldCoord(x, y);
    wc.w = 1;
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

ofVec2f KinectProjector::gradientAtKinectCoord(float x, float y){
    int ind = static_cast<int>(floor(x/gradFieldResolution)) + gradFieldcols*static_cast<int>(floor(y/gradFieldResolution));
    fishInd = ind;
    return gradField[ind];
}

void KinectProjector::setupGui(){
    // instantiate and position the gui //
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
    gui->addFRM();
    gui->addBreak();
    gui->addSlider("Tilt X", -30, 30, 0);
    gui->addSlider("Tilt Y", -30, 30, 0);
    gui->addSlider("Vertical offset", -100, 100, 0);
    gui->addButton("Reset sea level");
    gui->addBreak();
    
    auto advancedFolder = gui->addFolder("Advanced", ofColor::purple);
    advancedFolder->addToggle("Display kinect depth view", drawKinectView)->setName("Draw kinect depth view");
    advancedFolder->addSlider("Ceiling", -300, 300, 0);
    advancedFolder->addToggle("Spatial filtering", spatialFiltering);
    advancedFolder->addToggle("Quick reaction", followBigChanges);
    advancedFolder->addSlider("Averaging", 1, 40, numAveragingSlots)->setPrecision(0);
    advancedFolder->addBreak();
    advancedFolder->addButton("Calibrate")->setName("Full Calibration");
//	advancedFolder->addButton("Update ROI from calibration");
//    gui->addButton("Automatically detect sand region");
//    calibrationFolder->addButton("Manually define sand region");
//    gui->addButton("Automatically calibrate kinect & projector");
//    calibrationFolder->addButton("Manually calibrate kinect & projector");
    
//    gui->addBreak();
    gui->addHeader(":: Settings ::", false);
    
    // once the gui has been assembled, register callbacks to listen for component specific events //
    gui->onButtonEvent(this, &KinectProjector::onButtonEvent);
    gui->onToggleEvent(this, &KinectProjector::onToggleEvent);
    gui->onSliderEvent(this, &KinectProjector::onSliderEvent);

	// disactivate autodraw
	gui->setAutoDraw(false);
}

void KinectProjector::startFullCalibration(){
    calibrating = true;
    calibrationState = CALIBRATION_STATE_FULL_AUTO_CALIBRATION;
    fullCalibState = FULL_CALIBRATION_STATE_ROI_DETERMINATION;
    ROICalibState = ROI_CALIBRATION_STATE_INIT;
    confirmModal->setTitle("Full calibration");
    calibModal->setTitle("Full calibration");
    askToFlattenSand();
    ofLogVerbose("KinectProjector") << "startFullCalibration(): Starting full calibration" ;
}

void KinectProjector::startAutomaticROIDetection(){
    calibrating = true;
    calibrationState = CALIBRATION_STATE_ROI_AUTO_DETERMINATION;
    ROICalibState = ROI_CALIBRATION_STATE_INIT;
    ofLogVerbose("KinectProjector") << "onButtonEvent(): Finding ROI" ;
    confirmModal->setTitle("Detect sand region");
    calibModal->setTitle("Detect sand region");
    askToFlattenSand();
    ofLogVerbose("KinectProjector") << "startAutomaticROIDetection(): starting ROI detection" ;
}

void KinectProjector::startAutomaticKinectProjectorCalibration(){
    calibrating = true;
    calibrationState = CALIBRATION_STATE_PROJ_KINECT_AUTO_CALIBRATION;
    autoCalibState = AUTOCALIB_STATE_INIT_POINT;
    confirmModal->setTitle("Calibrate projector");
    calibModal->setTitle("Calibrate projector");
    askToFlattenSand();
    ofLogVerbose("KinectProjector") << "startAutomaticKinectProjectorCalibration(): Starting autocalib" ;
}

void KinectProjector::setSpatialFiltering(bool sspatialFiltering){
    spatialFiltering = sspatialFiltering;
    kinectgrabber.performInThread([sspatialFiltering](KinectGrabber & kg) {
        kg.setSpatialFiltering(sspatialFiltering);
    });
}

void KinectProjector::setFollowBigChanges(bool sfollowBigChanges){
    followBigChanges = sfollowBigChanges;
    kinectgrabber.performInThread([sfollowBigChanges](KinectGrabber & kg) {
        kg.setFollowBigChange(sfollowBigChanges);
    });
}

void KinectProjector::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Full Calibration")) {
        startFullCalibration();
    } else if (e.target->is("Update ROI from calibration")) {
		updateROIFromCalibration();
	} else if (e.target->is("Automatically detect sand region")) {
        startAutomaticROIDetection();
    } else if (e.target->is("Manually define sand region")){
        // Not implemented yet
    } else if (e.target->is("Automatically calibrate kinect & projector")){
        startAutomaticKinectProjectorCalibration();
    } else if (e.target->is("Manually calibrate kinect & projector")) {
        // Not implemented yet
    } else if (e.target->is("Reset sea level")){
        gui->getSlider("Tilt X")->setValue(0);
        gui->getSlider("Tilt Y")->setValue(0);
        gui->getSlider("Vertical offset")->setValue(0);
        basePlaneNormal = basePlaneNormalBack;
        basePlaneOffset = basePlaneOffsetBack;
        basePlaneEq = getPlaneEquation(basePlaneOffset,basePlaneNormal);
        basePlaneUpdated = true;
    }
}

void KinectProjector::onToggleEvent(ofxDatGuiToggleEvent e){
    if (e.target->is("Spatial filtering")) {
		setSpatialFiltering(e.checked);
    }else if (e.target->is("Quick reaction")) {
        setFollowBigChanges(e.checked);
    } else if (e.target->is("Draw kinect depth view")){
        drawKinectView = e.checked;
    }
}

void KinectProjector::onSliderEvent(ofxDatGuiSliderEvent e){
    if (e.target->is("Tilt X") || e.target->is("Tilt Y")) {
        basePlaneNormal = basePlaneNormalBack.getRotated(gui->getSlider("Tilt X")->getValue(), ofVec3f(1,0,0));
        basePlaneNormal.rotate(gui->getSlider("Tilt Y")->getValue(), ofVec3f(0,1,0));
        basePlaneEq = getPlaneEquation(basePlaneOffset,basePlaneNormal);
        basePlaneUpdated = true;
    } else if (e.target->is("Vertical offset")) {
        basePlaneOffset.z = basePlaneOffsetBack.z + e.value;
        basePlaneEq = getPlaneEquation(basePlaneOffset,basePlaneNormal);
        basePlaneUpdated = true;
    } else if (e.target->is("Ceiling")){
        maxOffset = maxOffsetBack-e.value;
        ofLogVerbose("KinectProjector") << "onSliderEvent(): maxOffset" << maxOffset ;
        kinectgrabber.performInThread([this](KinectGrabber & kg) {
            kg.setMaxOffset(this->maxOffset);
        });
    } else if(e.target->is("Averaging")){
        numAveragingSlots = e.value;
        kinectgrabber.performInThread([e](KinectGrabber & kg) {
            kg.setAveragingSlotsNumber(e.value);
        });
    }
}

void KinectProjector::onConfirmModalEvent(ofxModalEvent e){
    if (e.type == ofxModalEvent::SHOWN){
        ofLogVerbose("KinectProjector") << "Confirm modal window is open" ;
    }   else if (e.type == ofxModalEvent::HIDDEN){
        if (!projKinectCalibrated && !calibrating)
            startFullCalibration();
        if (calibrating)
            calibModal->show();
		if (!kinectOpened) {
			confirmModal->setMessage("Still no connection to Kinect. Please check that the kinect is (1) connected, (2) powerer and (3) not used by another application.");
			confirmModal->show();
		}
		ofLogVerbose("KinectProjector") << "Confirm modal window is closed" ;
    }   else if (e.type == ofxModalEvent::CANCEL){
        calibrating = false;
		kinectOpened = true; // The user don't care...
        ofLogVerbose("KinectProjector") << "Modal cancel button pressed: Aborting" ;
    }   else if (e.type == ofxModalEvent::CONFIRM){
        if (calibrating){
            if (waitingForFlattenSand){
                waitingForFlattenSand = false;
            }  else if ((calibrationState == CALIBRATION_STATE_PROJ_KINECT_AUTO_CALIBRATION || (calibrationState == CALIBRATION_STATE_FULL_AUTO_CALIBRATION && fullCalibState == FULL_CALIBRATION_STATE_AUTOCALIB))
                        && autoCalibState == AUTOCALIB_STATE_NEXT_POINT){
                if (!upframe){
                    upframe = true;
                }
            }
        }
		if (!kinectOpened) {
			kinectOpened = kinectgrabber.openKinect();
		}
        ofLogVerbose("KinectProjector") << "Modal confirm button pressed" ;
    }
}

void KinectProjector::onCalibModalEvent(ofxModalEvent e){
    if (e.type == ofxModalEvent::SHOWN){
        cout << "calib modal window is open" << endl;
    }   else if (e.type == ofxModalEvent::HIDDEN){
        cout << "calib modal window is closed" << endl;
    }   else if (e.type == ofxModalEvent::CONFIRM){
        calibrating = false;
        ofLogVerbose("KinectProjector") << "Modal cancel button pressed: Aborting" ;
    }
}

void KinectProjector::saveCalibrationAndSettings(){
    if (kpt->saveCalibration("settings/calibration.xml"))
    {
        ofLogVerbose("KinectProjector") << "update(): initialisation: Calibration saved " ;
    } else {
        ofLogVerbose("KinectProjector") << "update(): initialisation: Calibration could not be saved " ;
    }
    if (saveSettings())
    {
        ofLogVerbose("KinectProjector") << "update(): initialisation: Settings saved " ;
    } else {
        ofLogVerbose("KinectProjector") << "update(): initialisation: Settings could not be saved " ;
    }
}

bool KinectProjector::loadSettings(){
    string settingsFile = "settings/kinectProjectorSettings.xml";
    
    ofXml xml;
    if (!xml.load(settingsFile))
        return false;
    xml.setTo("KINECTSETTINGS");
    kinectROI = xml.getValue<ofRectangle>("kinectROI");
    basePlaneNormalBack = xml.getValue<ofVec3f>("basePlaneNormalBack");
    basePlaneNormal = basePlaneNormalBack;
    basePlaneOffsetBack = xml.getValue<ofVec3f>("basePlaneOffsetBack");
    basePlaneOffset = basePlaneOffsetBack;
    basePlaneEq = xml.getValue<ofVec4f>("basePlaneEq");
    maxOffsetBack = xml.getValue<float>("maxOffsetBack");
    maxOffset = maxOffsetBack;
    spatialFiltering = xml.getValue<bool>("spatialFiltering");
    followBigChanges = xml.getValue<bool>("followBigChanges");
    numAveragingSlots = xml.getValue<int>("numAveragingSlots");
    return true;
}

bool KinectProjector::saveSettings(){
    string settingsFile = "settings/kinectProjectorSettings.xml";

    ofXml xml;
    xml.addChild("KINECTSETTINGS");
    xml.setTo("KINECTSETTINGS");
    xml.addValue("kinectROI", kinectROI);
    xml.addValue("basePlaneNormalBack", basePlaneNormalBack);
    xml.addValue("basePlaneOffsetBack", basePlaneOffsetBack);
    xml.addValue("basePlaneEq", basePlaneEq);
    xml.addValue("maxOffsetBack", maxOffsetBack);
    xml.addValue("spatialFiltering", spatialFiltering);
    xml.addValue("followBigChanges", followBigChanges);
    xml.addValue("numAveragingSlots", numAveragingSlots);
    xml.setToParent();
    return xml.save(settingsFile);
}