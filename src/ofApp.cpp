#include "ofApp.h"

using namespace ofxCv;
using namespace cv;
using namespace ofxCSG;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
    //    ofSetBackgroundAuto(false); // Avoid autoclear on draw
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    
    // Get projector size
    projResX = projWindow->getWidth();
    projResY = projWindow->getHeight();
    
    kinectProjector = std::make_shared<KinectProjector>();
    
    kinectProjector->setup(projResX, projResY);
    
    sandSurfaceRendererer = new SandSurfaceRenderer(kinectProjector);
    
	// settings and defaults
	generalState = GENERAL_STATE_CALIBRATION;
    previousGeneralState = GENERAL_STATE_CALIBRATION;
	calibrationState  = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
    previousCalibrationState = CALIBRATION_STATE_PROJ_KINECT_CALIBRATION;
    ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
    autoCalibState = AUTOCALIB_STATE_INIT_FIRST_PLANE;
    initialisationState = INITIALISATION_STATE_DONE;
    waitingToInitialiseVehicles = false;
    
    // Vehicles
    fishNum = 1;
    rabbitsNum = 0;
    isMother = false;
    showMotherFish = false;
    showMotherRabbit = false;
    motherPlatformSize = 20;
    
    //    setupVehicles();
    
    fboProjWindow.allocate(projResX, projResY, GL_RGBA);
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();

    setupGui();
}


//--------------------------------------------------------------
void ofApp::setupVehicles(){
    // Resize vehicles vectors
    fish.resize(fishNum);
    rabbits.resize(rabbitsNum);
    
    waitingToInitialiseVehicles = false;
    
    // Check the relative earth and water ratio in the map
//    vhcle = FilteredDepthImage;
//    float elevation;
//    int earth = 0;
//    for (int x = kinectROI.getLeft(); x<kinectROI.getRight(); x ++){ // Project on base plane
//        for (int y = kinectROI.getTop(); y < kinectROI.getBottom(); y++){
//            ofVec4f vertexCc = getWorldCoord(x, y);
//            elevation = -basePlaneEq.dot(vertexCc);
//            if (elevation > 0){
//                vhcle.getPixels().getData()[y*kinectResX+x] = 255;
//                earth ++;
//            } else {
//                vhcle.getPixels().getData()[y*kinectResX+x] = 0;
//            }
//        }
//    }
//    float rapprt = (float)earth/(kinectROI.width*kinectROI.height);
//    ofLogVerbose("GreatSand") << "setupVehicles(): Earth/water ratio: "<< rapprt ;
    
    ofPoint location;
    bool outsidewater = false;
    bool insidewater = false;
    
    // Set the fish location at random places in the water
    for (auto & f : fish){
        insidewater = false;
        while (!insidewater) {
            location = ofPoint(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
            if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 0)
                insidewater = true;
        }
        f.setup(location.x, location.y, kinectROI);
        f.animalCoef = 1;
    }
    
    // Set the rabbits location at random places on the earth
    for (auto & r : rabbits){
        outsidewater = false;
        while (!outsidewater) {
            location = ofPoint(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
            if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 255)
                outsidewater = true;
        }
        r.setup(location.x, location.y, kinectROI);
        r.animalCoef = -1;
    }
    
    // If there are mothers for fish and rabbits, set their locations
    if (isMother){
        int minborderDist = 40;
        ofRectangle internalBorders = kinectROI;
        internalBorders.scaleFromCenter((kinectROI.width-minborderDist)/kinectROI.width, (kinectROI.height-minborderDist)/kinectROI.height);
        
        ofPoint close;
        ofVec4f vertexCc, wc;
        float seaLevelDistance = 10; // We set a plateform for the fish and rabbits mothers
        
        if (fishNum > 0) {
            // Find a location for the Fish mother outside of the water to be sure the fish cannot reach her without help
            outsidewater = false;
            while (!outsidewater) {
                location = ofPoint(ofRandom(internalBorders.getLeft(),internalBorders.getRight()), ofRandom(internalBorders.getTop(),internalBorders.getBottom()));
                if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 255)
                    outsidewater = true;
            }
            motherFish = location;
            
            // Set the mother Fish plateform location under the sea level
            wc = location;
            wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[((int)location.y*kinectResX)+(int)location.x];
            wc.w = 1;
            insidewater = false;
            while (!insidewater) {
                vertexCc = kinectWorldMatrix*wc*wc.z;
                vertexCc.w = 1;
                elevation = -basePlaneEq.dot(vertexCc);
                if (elevation < -seaLevelDistance){
                    insidewater = true;
                } else {
                    wc.z += 1; // Move down the plateform (kinect coordinates are reversed
                }
            }
            motherFish.z = wc.z;
        }
        if (rabbitsNum > 0){
            // Find a location for the Rabbits mother inside of the water to be sure the rabbits cannot reach her without help
            insidewater = false;
            while (!insidewater) {
                location = ofPoint(ofRandom(internalBorders.getLeft(),internalBorders.getRight()), ofRandom(internalBorders.getTop(),internalBorders.getBottom()));
                if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 0)
                    insidewater = true;
            }
            motherRabbit = location;
            
            // Set the Rabbits plateform location over the sea level
            wc = location;
            wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[((int)location.y*kinectResX)+(int)location.x];
            wc.w = 1;
            outsidewater = false;
            while (!outsidewater) {
                vertexCc = kinectWorldMatrix*wc*wc.z;
                vertexCc.w = 1;
                elevation = -basePlaneEq.dot(vertexCc);
                if (elevation > seaLevelDistance){
                    outsidewater = true;
                } else {
                    wc.z -= 1; // Move up the plateform (kinect coordinates are reversed
                }
            }
            motherRabbit.z = wc.z;
        }
    }
}

//--------------------------------------------------------------
void ofApp::update(){
    
	// Get depth image from kinect grabber
    ofFloatPixels filteredframe;
	if (kinectgrabber.filtered.tryReceive(filteredframe)) {
        
		FilteredDepthImage.setFromPixels(filteredframe.getData(), kinectResX, kinectResY);
		FilteredDepthImage.updateTexture();
        
        // Update grabber stored frame number
		kinectgrabber.lock();
		kinectgrabber.decStoredframes();
		kinectgrabber.unlock();
        
        // Is the depth image stabilized
        if (kinectgrabber.isFirstImageReady())
            firstImageReady = true;
        
        if (generalState == GENERAL_STATE_CALIBRATION) {
            // Get color image from kinect grabber
            ofPixels coloredframe;
            if (kinectgrabber.colored.tryReceive(coloredframe)) {
                kinectColorImage.setFromPixels(coloredframe);
            }
            
            if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
                // Draw a testing point
                ofVec2f t = ofVec2f(min((float)kinectResX-1,testPoint.x), min((float)kinectResY-1,testPoint.y));
                ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));
                drawTestingPoint(projectedPoint);
            }
            else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION) {
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
            } else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION){
                // Update ROI auto calibration state
                updateROIAutoSetup();
            } else if (calibrationState == CALIBRATION_STATE_AUTOCALIB){
                // Update autocalibr state
                autoCalib();
            } else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP){
                // Update ROI calibration state
                if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
                    kinectROIManualCalib.setSize(ofGetMouseX()-kinectROIManualCalib.x,ofGetMouseY()-kinectROIManualCalib.y);
                }
                updateROIManualSetup();
            }
        }
        else if (generalState == GENERAL_STATE_SANDBOX){
            // Prepare contour line if needed
            if (drawContourLines)
                prepareContourLines();
            // Draw the sandbox on the fbo
            drawSandbox();
        }
        else if (generalState == GENERAL_STATE_GAME1){
            // Get gradient field from kinect grabber
            kinectgrabber.gradient.tryReceive(gradField);
            // Prepare contour line if needed
            if (drawContourLines)
                prepareContourLines();
            // Draw the sandbox on the fbo
            drawSandbox();
            
            //drawFlowField();
        }
    }
    if (generalState == GENERAL_STATE_GAME1){
        // update vehicles
        if (firstImageReady) {
            // Are we waiting for a stable acquisition to initialize vehicles locations ?
            if (waitingToInitialiseVehicles)
                setupVehicles();
            
            // update vehicles positions
            for (auto & f : fish){
                updateVehiclesFutureElevationAndGradient(f);
                f.applyBehaviours(motherFish);
                f.update();
            }
            for (auto & r : rabbits){
                updateVehiclesFutureElevationAndGradient(r);
                r.applyBehaviours(motherRabbit);
                r.update();
            }
        }
    }
    if (initialisationState == INITIALISATION_STATE_ROI_DETERMINATION){
        modaltext = "Please wait while sand area is being detected";
        if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE) {
            initialisationState = INITIALISATION_STATE_AUTOCALIB;
            calibrationState = CALIBRATION_STATE_AUTOCALIB;
            autoCalibState = AUTOCALIB_STATE_INIT_FIRST_PLANE;
        }
    } else if (initialisationState ==INITIALISATION_STATE_AUTOCALIB){
        if (autoCalibState == AUTOCALIB_STATE_DONE){
            initialisationState = INITIALISATION_STATE_DONE;
            generalState = GENERAL_STATE_SANDBOX;
            
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
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
//    if (generalState == GENERAL_STATE_CALIBRATION) {
////        ofSetColor(0);
//        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
//            kinectColorImage.draw(0, 0, 640, 480);
//            FilteredDepthImage.draw(650, 0, 320, 240);
//            
////            ofNoFill();
////            ofSetColor(255);
////            ofDrawRectangle(kinectROI);
////            ofFill();
////            
////            ofDrawBitmapStringHighlight("Click on the image to test a point in the RGB image.", 340, 510);
////            ofDrawBitmapStringHighlight("The projector should place a green dot on the corresponding point.", 340, 530);
////            ofSetColor(255, 0, 0);
////            
//            //Draw testing point indicator
//            float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 40);
//            ofDrawCircle(testPoint.x, testPoint.y, ptSize);
//            
//        } else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
//        {
//            kinectColorImage.draw(0, 0, 640, 480);
//            FilteredDepthImage.draw(650, 0, 320, 240);
//            
//            ofNoFill();
//            ofSetColor(255);
//            ofDrawRectangle(kinectROI);
//            ofFill();
//            
//        } else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION)
//        {
//            ofSetColor(255);
//            thresholdedImage.draw(0, 0, 640, 480);
//            //            FilteredDepthImage.draw(650, 0, 320, 240);
//            //
//            ofNoFill();
//            ofSetColor(255);
//            ofDrawRectangle(kinectROI);
//            ofFill();
//            
//            //           ofSetColor(255);
//            //            thresholdedImage.draw(650, 0, 320, 240); // Overwrite depth image
//            contourFinder.draw(0, 0);//, 320, 240); // Draw contour finder results
//            large.draw();
//        } else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP && ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
//            ofNoFill();
//            ofSetColor(0,255,0,255);
//            ofDrawRectangle(kinectROIManualCalib);
//            ofFill();
//        }
//        ofSetColor(255);
//    } else if (generalState == GENERAL_STATE_SANDBOX){
//        fboProjWindow.draw(0, 0, 640, 480);
//        //Draw testing point indicator
//        float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 10);
//        ofDrawCircle(testPoint.x, testPoint.y, ptSize);
//        
//        ofRectangle imgROI;
//        imgROI.setFromCenter(testPoint, 20, 20);
//        kinectColorImage.setROI(imgROI);
//        kinectColorImage.drawROI(650, 10, 100, 100);
//        ofDrawCircle(700, 60, ptSize);
//        
//        if (firstImageReady) {
//            //            FilteredDepthImage.setROI(imgROI);
//            float * roi_ptr = (float*)FilteredDepthImage.getFloatPixelsRef().getData() + ((int)(imgROI.y)*kinectResX) + (int)imgROI.x;
//            ofFloatPixels ROIDpt;
//            ROIDpt.setNumChannels(1);
//            ROIDpt.setFromAlignedPixels(roi_ptr,imgROI.width,imgROI.height,1,kinectResX*4);
//            Dptimg.setFromPixels(ROIDpt);
//            
//            Dptimg.setNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);
//            Dptimg.contrastStretch();
//            Dptimg.draw(650, 120, 100, 100);
//            ofDrawCircle(700, 170, ptSize);
//        }
//    } else if (generalState == GENERAL_STATE_GAME1) {
//        vhcle.draw(0,0);
//        contourFinder.draw(0,0);
//    }
//    
    fboProjWindow.draw(0, 0, 640, 480);
    ofNoFill();
    ofSetColor(255);
    ofDrawRectangle(kinectROI);
    ofFill();
    //    drawTextGui();
//    if (showModal){
//        modal.draw();
//    } else {
//        calibration.draw();
//        animals.draw();
//        sealevel.draw();
//        display.draw();
//    }
    
}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){
    // Main draw call for projector window
    fboProjWindow.draw(0, 0);
    
    if (generalState == GENERAL_STATE_GAME1)
        drawVehicles();
}

//--------------------------------------------------------------
void ofApp::updateVehiclesFutureElevationAndGradient(vehicle& v){
    // Find sandbox gradients and elevations in the max 10 next steps of vehicle v, update vehicle variables
    std::vector<float> futureZ;
    std::vector<ofVec2f> futureGradient;
    
    ofPoint futureLocation, velocity;
    
    futureLocation = v.getLocation();
    velocity = v.getVelocity();
    futureZ.clear();
    futureGradient.clear();
    int i = 1;
    float beachDist = 1;
    ofVec2f beachSlope(0);
    bool water = true;
    while (i < 10 && water)
    {
        ofVec4f wc = ofVec3f(futureLocation);
        float elevation = -basePlaneEq.dot(getWorldCoord(futureLocation.x, futureLocation.y));
        if (v.animalCoef*elevation > 0)
        {
            water = false;
            beachDist = i;
            beachSlope = -v.animalCoef*gradField[(int)(futureLocation.x/gradFieldresolution)+gradFieldcols*((int)(futureLocation.y/gradFieldresolution))];
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
    // Store these values in the vehicle
    v.updateBeachDetection(!water, beachDist, beachSlope);
}

//--------------------------------------------------------------
void ofApp::drawVehicles()
{
    if (isMother){
        drawMotherFish();
        drawMotherRabbit();
    }
    for (auto & f : fish){
        // Move to vehicule projected location and angle
        ofVec2f t = f.getLocation();
        ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));
        float angle = f.getAngle();
        ofPushMatrix();
        ofTranslate(projectedPoint);
        ofRotate(angle);
        
        // Draw fish
        f.draw();
        
        ofPopMatrix();
    }
    for (auto & r : rabbits){
        // Move to vehicule projected location and angle
        ofVec2f t = r.getLocation();
        ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));
        float angle = r.getAngle();
        ofPushMatrix();
        ofTranslate(projectedPoint);
        ofRotate(angle);
        
        // Draw rabbit
        r.draw();
        
        ofPopMatrix();
    }
}

//--------------------------------------------------------------
void ofApp::drawMotherFish()//, std::vector<ofVec2f> forces)
{
    // Get vehicule coord
    ofVec2f t = motherFish;
    ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));//kpt.getProjectedPoint(worldPoint);
    float tailangle = 0;//nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    
    float sc = 10; // Mother fish scale
    
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    
    projectedPoint.x +=tailSize;
    
    ofNoFill();
    ofPushMatrix();
    ofTranslate(projectedPoint);
    
    ofSetColor(255);
    ofPolyline fish;
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8)));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8)));
    fish.curveTo( ofPoint(-fishLength, 0));
    fish.curveTo( ofPoint(0, -fishHead));
    fish.curveTo( ofPoint(fishHead, 0));
    fish.curveTo( ofPoint(0, fishHead));
    fish.curveTo( ofPoint(-fishLength, 0));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8)));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8)));
    fish.close();
    ofSetLineWidth(2.0);  // Line widths apply to polylines
    fish.draw();
    ofSetColor(255);
    ofDrawCircle(0, 0, 5);
    
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::drawMotherRabbit()//, std::vector<ofVec2f> forces)
{
    // Get vehicule coord
    ofVec2f t = motherRabbit;
    ofVec2f projectedPoint = computeTransform(getWorldCoord(t.x, t.y));//kpt.getProjectedPoint(worldPoint);
    
    ofPushMatrix();
    
    float sc = 2; // MotherRabbit scale
    
    projectedPoint.x += 5*sc;
    
    ofTranslate(projectedPoint);
    
    ofFill();
    ofSetLineWidth(1.0);  // Line widths apply to polylines
    
    ofPath body;
    body.curveTo( ofPoint(-2*sc, 5.5*sc));
    body.curveTo( ofPoint(-2*sc, 5.5*sc));
    body.curveTo( ofPoint(-9*sc, 7.5*sc));
    body.curveTo( ofPoint(-17*sc, 0*sc));
    body.curveTo( ofPoint(-9*sc, -7.5*sc));
    body.curveTo( ofPoint(-2*sc, -5.5*sc));
    body.curveTo( ofPoint(4*sc, 0*sc));
    body.curveTo( ofPoint(4*sc, 0*sc));
    body.close();
    //    ofSetLineWidth(2.0);  // Line widths apply to polylines
    body.setFillColor(0);
    body.draw();
    
    ofSetColor(255);
    ofDrawCircle(-19*sc, 0, 2*sc);
    
    ofPath head;
    head.curveTo( ofPoint(0, 1.5*sc));
    head.curveTo( ofPoint(0, 1.5*sc));
    head.curveTo( ofPoint(-3*sc, 1.5*sc));
    head.curveTo( ofPoint(-9*sc, 3.5*sc));
    head.curveTo( ofPoint(0, 5.5*sc));
    head.curveTo( ofPoint(8*sc, 0));
    head.curveTo( ofPoint(0, -5.5*sc));
    head.curveTo( ofPoint(-9*sc, -3.5*sc));
    head.curveTo( ofPoint(-3*sc, -1.5*sc));
    head.curveTo( ofPoint(0, -1.5*sc));
    head.curveTo( ofPoint(0, -1.5*sc));
    head.close();
    head.setFillColor(255);
    head.draw();
    
    ofSetColor(0);
    ofDrawCircle(8.5*sc, 0, 1*sc);
    
    ofPopMatrix();
    ofSetColor(255);
}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key==' '){
            tIndex = tIndex < themes.size()-1 ? tIndex+1 : 0;
            gui->setTheme(themes[tIndex]);
 //        if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
//        {
//            ofLogVerbose("GreatSand") << "keyPressed(): Adding point pair" ;
//            addPointPair();
//        } else if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_AUTOCALIB && autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE)
//        {
//            autoCalibState = AUTOCALIB_STATE_INIT_POINT;
//        } else if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_AUTOCALIB && autoCalibState == AUTOCALIB_STATE_NEXT_POINT) {
//            if (!upframe)
//                upframe = true;
//        } else if (generalState == GENERAL_STATE_GAME1)
//        {
//            setupVehicles();
//        }
        //        if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_ROI_DETERMINATION && ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP)
        //        {
        //            ofLogVerbose("GreatSand") << "keyPressed(): Changing threshold : " << threshold ;
        //            threshold+=1;
        //        }
    } else if (key=='a') {
        chessboardSize -= 20;
    } else if (key=='z') {
        chessboardSize += 20;
    } else if (key=='q') {
        basePlaneOffset.z += 0.5;
        setRangesAndBasePlaneEquation();
    } else if (key=='s') {
        basePlaneOffset.z -= 0.5;
        setRangesAndBasePlaneEquation();
    } else if (key=='g') {
        contourLineDistance += 1.0;
        setRangesAndBasePlaneEquation();
    } else if (key=='h') {
        contourLineDistance -= 1.0;
        setRangesAndBasePlaneEquation();
    } else if (key=='w') {
        maxOffset += 0.5;
        ofLogVerbose("GreatSand") << "keyPressed(): maxOffset" << maxOffset ;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if (key=='x') {
        maxOffset -= 0.5;
        ofLogVerbose("GreatSand") << "keyPressed(): maxOffset" << maxOffset ;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if (key=='d') {
        //        doThemeColorsWindow = !doThemeColorsWindow;
        //        ofLogVerbose("GreatSand") << "doThemeColorsWindow: " << doThemeColorsWindow ;
    } else if (key=='f') {
        //        doSetTheme = !doSetTheme;
        //        ofLogVerbose("GreatSand") << "doSetTheme: " << doSetTheme ;
    } else if (key=='u') {
        basePlaneNormal.rotate(-1, ofVec3f(1,0,0)); // Rotate the base plane normal
        setRangesAndBasePlaneEquation();
    } else if (key=='i') {
        basePlaneNormal.rotate(1, ofVec3f(1,0,0)); // Rotate the base plane normal
        setRangesAndBasePlaneEquation();
    } else if (key=='o') {
        basePlaneNormal.rotate(-1, ofVec3f(0,1,0)); // Rotate the base plane normal
        setRangesAndBasePlaneEquation();
    } else if (key=='p') {
        basePlaneNormal.rotate(1, ofVec3f(0,1,0)); // Rotate the base plane normal
        setRangesAndBasePlaneEquation();
    } else if (key=='n') {
        computeBasePlane();
    } else if (key=='c') {
        if (pairsKinect.size() == 0) {
            ofLogVerbose("GreatSand") << "keyPressed(): Cannot calibrate: No points acquired !!" ;
        } else {
            ofLogVerbose("GreatSand") << "keyPressed(): calibrating" ;
            kpt.calibrate(pairsKinect, pairsProjector);
            kinectProjMatrix = kpt.getProjectionMatrix();
            saved = false;
            loaded = false;
            calibrated = true;
        }
    } else if (key=='r') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "keyPressed(): Finding ROI" ;
    } else if (key=='m') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_MANUAL_SETUP;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "keyPressed(): Updating ROI Manually" ;
    } else if (key=='y') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_AUTOCALIB;
        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        ofLogVerbose("GreatSand") << "keyPressed(): Starting autocalib" ;
    } else if (key=='t') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
    } else if (key=='b') {
        generalState = GENERAL_STATE_SANDBOX;
    }else if (key=='e') {
        waitingToInitialiseVehicles = true;
        generalState = GENERAL_STATE_GAME1;
    } else if (key=='v') {
        if (kpt.saveCalibration("calibration.xml"))
        {
            ofLogVerbose("GreatSand") << "keyPressed(): Calibration saved " ;
            saved = true;
        } else {
            ofLogVerbose("GreatSand") << "keyPressed(): Calibration could not be saved " ;
        }
    } else if (key=='l') {
        if (kpt.loadCalibration("calibration.xml"))
        {
            ofLogVerbose("GreatSand") << "keyPressed(): Calibration loaded " ;
            kinectProjMatrix = kpt.getProjectionMatrix();
            loaded = true;
            calibrated = true;
        } else {
            ofLogVerbose("GreatSand") << "keyPressed(): Calibration could not be loaded " ;
        }
    } else if (key=='j') {
        if (saveSettings("settings.xml"))
        {
            ofLogVerbose("GreatSand") << "keyPressed(): Settings saved " ;
        } else {
            ofLogVerbose("GreatSand") << "keyPressed(): Settings could not be saved " ;
        }
    } else if (key=='k') {
        if (loadSettings("settings.xml"))
        {
            ofLogVerbose("GreatSand") << "keyPressed(): Settings loaded " ;
            setRangesAndBasePlaneEquation();
            updateKinectGrabberROI();
        } else {
            ofLogVerbose("GreatSand") << "keyPressed(): Settings could not be loaded " ;
        }
    }
    
    if (key=='r'|| key== 'm' || key== 'y'|| key=='t' || key=='b' || key == 'e') {
        firstImageReady = false;
        updateMode();
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
    //    if (generalState == GENERAL_STATE_SANDBOX || (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_CALIBRATION_TEST)) {
    //        if (x<kinectResX && y <kinectResY) {
    //            testPoint.set(min(x, kinectResX-1), min(y, kinectResY-1));
    //
    //            int idx = (int)testPoint.x+kinectResX*(int)testPoint.y;
    //            ofLogVerbose("GreatSand") << "Depth value at point: " << FilteredDepthImage.getFloatPixelsRef().getData()[idx];
    //            float* sPtr=kinectgrabber.framefilter.statBuffer+3*idx;
    //            ofLogVerbose("GreatSand") << " Number of valid samples statBuffer[0]: " << sPtr[0] ;
    //            ofLogVerbose("GreatSand") << " Sum of valid samples statBuffer[1]: " << sPtr[1] ; //
    //            ofLogVerbose("GreatSand") << " Sum of squares of valid samples statBuffer[2]: " << sPtr[2] ; // Sum of squares of valid samples;
    //        } else if (x > 650 && x < 750 && y > 120 && y < 220) {
    //            ofVec2f tmp = testPoint;
    //            testPoint.set(tmp.x+(x-700)/5, tmp.y+(y-170)/5);
    //
    //            int idx = (int)testPoint.x+kinectResX*(int)testPoint.y;
    //            ofLogVerbose("GreatSand") << "Depth value at point: " << FilteredDepthImage.getFloatPixelsRef().getData()[idx];
    //            float* sPtr=kinectgrabber.framefilter.statBuffer+3*idx;
    //            ofLogVerbose("GreatSand") << " Number of valid samples statBuffer[0]: " << sPtr[0] ;
    //            ofLogVerbose("GreatSand") << " Sum of valid samples statBuffer[1]: " << sPtr[1] ; //
    //            ofLogVerbose("GreatSand") << " Sum of squares of valid samples statBuffer[2]: " << sPtr[2] ; // Sum of squares of valid samples;
    //        }
    //    }
    if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP){
        // Manual ROI calibration
        if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT)
        {
            kinectROIManualCalib.setPosition(x, y);
            ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
        } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
            kinectROIManualCalib.setSize(x-kinectROIManualCalib.x, y-kinectROIManualCalib.y);
            ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
            
            kinectROI = kinectROIManualCalib;
            kinectROI.standardize();
            ofLogVerbose("GreatSand") << "mousePressed(): kinectROI : " << kinectROI ;
            updateKinectGrabberROI();
            //update the mesh
            setupMesh();
        }
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

//--------------------------------------------------------------
void ofApp::setupGui(){
    
    // instantiate and position the gui //
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
    
    // add some components //
//    gui->addTextInput("message", "# open frameworks #");
    
    gui->addFRM();
    gui->addBreak();
    
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* sealevelFolder = gui->addFolder("Sea level settings", ofColor::blueSteel);
    sealevelFolder->addSlider("Tilt X", -30, 30, 0);
    sealevelFolder->addSlider("Tilt Y", -30, 30, 0);
    sealevelFolder->addSlider("Vertical offset", heightMap.getScalarRangeMin()*.5, heightMap.getScalarRangeMax()*.5, 0);
    sealevelFolder->addButton("Reset sea level");
    sealevelFolder->expand();

    gui->addBreak();
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* displayFolder = gui->addFolder("Display settings", ofColor::orangeRed);
    displayFolder->addToggle("Display contourlines", drawContourLines);
    displayFolder->addSlider("Contourlines\ndistance", 1, 100, contourLineDistance);
    displayFolder->addSlider("Highest detection\nlevel", 0, 100, maxOffset);
    displayFolder->addToggle("Spatial filtering", true);
    displayFolder->addToggle("Quick reaction (follow hands)", true);
    displayFolder->addSlider("# of averaging slots", 0, 40, 30)->setPrecision(0);
    displayFolder->expand();

    gui->addBreak();
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* animalFolder = gui->addFolder("Animals settings", ofColor::greenYellow);
    animalFolder->addSlider("# of fish", 0, 10, 0)->setPrecision(0);
    animalFolder->addSlider("# of rabbits", 0, 10, 0)->setPrecision(0);
    animalFolder->addToggle("Mother fish", showMotherFish);
    animalFolder->addToggle("Mother rabbit", showMotherRabbit);
    animalFolder->addButton("Reset animal locations");
    animalFolder->addButton("Remove all animals");
    
    // add a folder to group a few components together //
    ofxDatGuiFolder* calibrationFolder = gui->addFolder("Calibration", ofColor::purple);
    calibrationFolder->addButton("Full Calibration");
    calibrationFolder->addButton("Automatically detect sand region");
    calibrationFolder->addButton("Manually define sand region");
    calibrationFolder->addButton("Automatically calibrate kinect & projector");
    calibrationFolder->addButton("Manually calibrate kinect & projector");
    calibrationFolder->addButton("Test kinect & projector calibration");
    // let's have it open by default. note: call this only after you're done adding items //
    //    calibrationFolder->expand();
    
    gui->addBreak();

    // adding the optional header allows you to drag the gui around //
    gui->addHeader(":: Settings ::");
    
    // adding the optional footer allows you to collapse/expand the gui //
    gui->addFooter();
    
    // once the gui has been assembled, register callbacks to listen for component specific events //
    gui->onButtonEvent(this, &ofApp::onButtonEvent);
    gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->onSliderEvent(this, &ofApp::onSliderEvent);
    
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
//--------------------------------------------------------------
void ofApp::onModalEvent(ofxModalEvent e){
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

//--------------------------------------------------------------
void ofApp::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Full Calibration")) {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        initialisationState = INITIALISATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Starting initialisation" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Automatically detect sand region")) {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Finding ROI" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Manually define sand region")){
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_MANUAL_SETUP;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Updating ROI Manually" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Automatically calibrate kinect & projector")){
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_AUTOCALIB;
        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Starting autocalib" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Manually calibrate kinect & projector")) {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Manual calibration" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Test kinect & projector calibration")) {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
        ofLogVerbose("GreatSand") << "onButtonEvent(): Texting calibration" ;
//        showModal = true;
        updateMode();
    } else if (e.target->is("Reset sea level")){
        basePlaneNormal = basePlaneNormalBack;
        basePlaneOffset.z = basePlaneOffsetBack.z;
        gui->getSlider("Tilt X")->setValue(0);
        gui->getSlider("Tilt Y")->setValue(0);
        gui->getSlider("Vertical offset")->setValue(0);
        setRangesAndBasePlaneEquation();
    }
}

//--------------------------------------------------------------
void ofApp::onToggleEvent(ofxDatGuiToggleEvent e){
    if (e.target->is("Display contourlines")) {
        drawContourLines = e.checked;
    } else if (e.target->is("Spatial filtering")) {
        kinectgrabber.setSpatialFiltering(e.checked);
    }else if (e.target->is("Quick reaction (follow hands)")) {
        kinectgrabber.setFollowBigChange(e.checked);
    }
}

//--------------------------------------------------------------
void ofApp::onSliderEvent(ofxDatGuiSliderEvent e){
    if (e.target->is("Tilt X") || e.target->is("Tilt Y")) {
        basePlaneNormal = basePlaneNormalBack.getRotated(gui->getSlider("Tilt X")->getValue(), ofVec3f(1,0,0));
        basePlaneNormal.rotate(gui->getSlider("Tilt Y")->getValue(), ofVec3f(0,1,0));
        setRangesAndBasePlaneEquation();
    } else if (e.target->is("Vertical offset")) {
        basePlaneOffset.z = basePlaneOffsetBack.z + e.value;
        setRangesAndBasePlaneEquation();
    } else if (e.target->is("Highest detection\nlevel")){
        maxOffset = e.value;
        ofLogVerbose("GreatSand") << "onSliderEvent(): maxOffset" << maxOffset ;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if (e.target->is("Contourlines\ndistance")){
        contourLineDistance = e.value;
        setRangesAndBasePlaneEquation();
    } else if(e.target->is("# of averaging slots")){
#if __cplusplus>=201103
        kinectgrabber.numAveragingSlotschannel.send(std::move(e.value));
#else
        kinectgrabber.numAveragingSlotschannel.send(e.value);
#endif
    }
    
   
}



