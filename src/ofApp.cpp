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
    waitingToInitialiseVehicles = false;
    
    // calibration chessboard config
	chessboardSize = 300;
	chessboardX = 5;
    chessboardY = 4;
    
    // 	gradFieldresolution
	gradFieldresolution = 10;
    arrowLength = 25;
	
    // Setup sandbox boundaries, base plane and kinect clip planes
	basePlaneNormal = ofVec3f(0,0,1);
	basePlaneOffset= ofVec3f(0,0,870);
    maxOffset = 0;
    maxOffsetSafeRange = 50; // Range above the autocalib measured max offset
    
    // Sandbox contourlines
    drawContourLines = true; // Flag if topographic contour lines are enabled
	contourLineDistance = 10.0; // Elevation distance between adjacent topographic contour lines in millimiters
    
    // Vehicles
    fishNum = 1;
    rabbitsNum = 0;
    isMother = false;
    motherPlatformSize = 20;
    
    // Load colormap and set heightmap
    heightMap.load("HeightColorMap.yml");
    
    // kinectgrabber: start
	kinectgrabber.setup(generalState, calibrationState);
    
    // Get projector and kinect width & height
    ofVec2f kinSize = kinectgrabber.getKinectSize();
    kinectResX = kinSize.x;
    kinectResY = kinSize.y;
	projResX =projWindow->getWidth();
	projResY =projWindow->getHeight();
	kinectROI = ofRectangle(0, 0, kinectResX, kinectResY);
    testPoint.set(kinectResX/2, kinectResY/2);
    
    // Initialize the fbos and images
    fboProjWindow.allocate(projResX, projResY, GL_RGBA);
    contourLineFramebufferObject.allocate(projResX+1, projResY+1, GL_RGBA);
    FilteredDepthImage.allocate(kinectResX, kinectResY);
    kinectColorImage.allocate(kinectResX, kinectResY);
    thresholdedImage.allocate(kinectResX, kinectResY);
    Dptimg.allocate(20, 20); // Small detailed ROI
    
    //setup the mesh
    setupMesh();
    
	// Load shaders
    bool loaded = true;
#ifdef TARGET_OPENGLES
    ofLogVerbose("GreatSand") << "setup(): Loading shadersES2";
	loaded = loaded && elevationShader.load("shadersES2/elevationShader");
	loaded = loaded && heightMapShader.load("shadersES2/heightMapShader");
#else
	if(ofIsGLProgrammableRenderer()){
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL3/elevationShader";
		loaded = loaded && elevationShader.load("shadersGL3/elevationShader");
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL3/heightMapShader";
		loaded = loaded && heightMapShader.load("shadersGL3/heightMapShader");
	}else{
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL2/elevationShader";
		loaded = loaded && elevationShader.load("shadersGL2/elevationShader");
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL2/heightMapShader";
		loaded = loaded && heightMapShader.load("shadersGL2/heightMapShader");
	}
#endif
    if (!loaded)
    {
        ofLogError("GreatSand") << "setup(): shader not loaded" ;
        exit();
    }
    
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
    
    // Setup elevation ranges and base plane equation
    setRangesAndBasePlaneEquation();
    
    // Setup gradient field
    setupGradientField();
    
    //    setupVehicles();
    
    setupGui();
    
	kinectgrabber.start(); // Start the acquisition
}


//--------------------------------------------------------------
void ofApp::clearFbos(){
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();
    
    contourLineFramebufferObject.begin();
    ofClear(0,0,0,255);
    contourLineFramebufferObject.end();
}

//--------------------------------------------------------------
void ofApp::setupMesh(){
    //clear Fbos
    clearFbos();
    
    // Initialise mesh
    //    float planeScale = 1;
    meshwidth = kinectROI.width;//kinectResX;
    meshheight = kinectROI.height;//kinectResY;
    mesh.clear();
    for(unsigned int y=0;y<meshheight;y++)
        for(unsigned int x=0;x<meshwidth;x++)
        {
            ofPoint pt = ofPoint(x*kinectResX/(meshwidth-1)+kinectROI.x,y*kinectResY/(meshheight-1)+kinectROI.y,0.0f)-ofPoint(0.5,0.5,0); // We move of a half pixel to center the color pixel (more beautiful)
            //        ofPoint pt = ofPoint(x*kinectResX*planeScale/(meshwidth-1)+kinectROI.x,y*kinectResY*planeScale/(meshheight-1)+kinectROI.y,0.0f)-kinectROI.getCenter()*planeScale+kinectROI.getCenter()-ofPoint(0.5,0.5,0); // with a planescaling
            mesh.addVertex(pt); // make a new vertex
            mesh.addTexCoord(pt);
        }
    for(unsigned int y=0;y<meshheight-1;y++)
        for(unsigned int x=0;x<meshwidth-1;x++)
        {
            mesh.addIndex(x+y*meshwidth);         // 0
            mesh.addIndex((x+1)+y*meshwidth);     // 1
            mesh.addIndex(x+(y+1)*meshwidth);     // 10
            
            mesh.addIndex((x+1)+y*meshwidth);     // 1
            mesh.addIndex((x+1)+(y+1)*meshwidth); // 11
            mesh.addIndex(x+(y+1)*meshwidth);     // 10
        }
}

//--------------------------------------------------------------
void ofApp::setRangesAndBasePlaneEquation(){
    //homogeneous base plane equation
    basePlaneEq=getPlaneEquation(basePlaneOffset,basePlaneNormal);
    
    //Set elevation Min and Max
    elevationMin=-heightMap.getScalarRangeMin();
    elevationMax=-heightMap.getScalarRangeMax();
    
    // Calculate the  height map elevation scaling and offset coefficients
	heightMapScale =(heightMap.getNumEntries()-1)/((elevationMax-elevationMin));
	heightMapOffset =0.5/heightMap.getNumEntries()-heightMapScale*elevationMin;
    
    // Set the FilteredDepthImage native scale - converted to 0..1 when send to the shader
    FilteredDepthImage.setNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);
    
    // Calculate the  FilteredDepthImage scaling and offset coefficients
	FilteredDepthScale = elevationMin-elevationMax;
	FilteredDepthOffset = basePlaneOffset.z+elevationMax;
    
    // Calculate the contourline fbo scaling and offset coefficients
	contourLineFboScale = elevationMin-elevationMax;
	contourLineFboOffset = elevationMax;
    contourLineFactor = contourLineFboScale/(contourLineDistance);
    
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneOffset: " << basePlaneOffset ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneNormal: " << basePlaneNormal ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneEq: " << basePlaneEq ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMin: " << elevationMin ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMax: " << elevationMax ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): heightMap.getNumEntries(): " << heightMap.getNumEntries() ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): contourLineDistance: " << contourLineDistance ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): maxOffset: " << maxOffset ;
}

//--------------------------------------------------------------
void ofApp::setupGradientField(){
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
void ofApp::setupVehicles(){
    // Resize vehicles vectors
    fish.resize(fishNum);
    rabbits.resize(rabbitsNum);
    
    waitingToInitialiseVehicles = false;
    
    // Check the relative earth and water ratio in the map
    vhcle = FilteredDepthImage;
    float elevation;
    int earth = 0;
    for (int x = kinectROI.getLeft(); x<kinectROI.getRight(); x ++){ // Project on base plane
        for (int y = kinectROI.getTop(); y < kinectROI.getBottom(); y++){
            ofVec4f vertexCc = getWorldCoord(x, y);
            elevation = -basePlaneEq.dot(vertexCc);
            if (elevation > 0){
                vhcle.getPixels().getData()[y*kinectResX+x] = 255;
                earth ++;
            } else {
                vhcle.getPixels().getData()[y*kinectResX+x] = 0;
            }
        }
    }
    float rapprt = (float)earth/(kinectROI.width*kinectROI.height);
    ofLogVerbose("GreatSand") << "setupVehicles(): Earth/water ratio: "<< rapprt ;
    
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
        mainMessage = "Please wait while sand area is being detected";
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
    // Draw the GUI window
    int ybase = 250;
    int yinc = 20;
    int i = 0;
    int xbase = 650;
    ofDrawBitmapStringHighlight("Position the chessboard using the mouse.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("[SPACE]: add point pair.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("a & z: inc/dec chessboard size.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("q & s: inc/dec baseplane offset.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("w & x: inc/dec maxOffset.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("d & f: change GUI doThemeColorsWindow & doSetTheme.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("g & h: inc/dec contour line distance.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("u, i, o & p: rotate baseplane normal.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("n: Find and compute base plane in ROI.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("c: compute calibration matrix.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("r: go to find kinect ROI mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("m: go to manual ROI setup mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("t: go to point test mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("y: go to autocalib mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("b: go to sandbox mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("e: go to game 1 mode.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("v: save calibration.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("l: load calibration.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("j: save settings.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight("k: load setting.", xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight(resultMessage, xbase, ybase+i*yinc);
    i++;
    ofDrawBitmapStringHighlight(ofToString(pairsKinect.size())+" point pairs collected.", xbase, ybase+i*yinc);
    
    if (generalState == GENERAL_STATE_CALIBRATION) {
        ofSetColor(0);
        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
            kinectColorImage.draw(0, 0, 640, 480);
            FilteredDepthImage.draw(650, 0, 320, 240);
            
            ofNoFill();
            ofSetColor(255);
            ofDrawRectangle(kinectROI);
            ofFill();
            
            ofDrawBitmapStringHighlight("Click on the image to test a point in the RGB image.", 340, 510);
            ofDrawBitmapStringHighlight("The projector should place a green dot on the corresponding point.", 340, 530);
            ofSetColor(255, 0, 0);
            
            //Draw testing point indicator
            float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 40);
            ofDrawCircle(testPoint.x, testPoint.y, ptSize);
            
        } else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
        {
            kinectColorImage.draw(0, 0, 640, 480);
            FilteredDepthImage.draw(650, 0, 320, 240);
            
            ofNoFill();
            ofSetColor(255);
            ofDrawRectangle(kinectROI);
            ofFill();
            
        } else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION)
        {
            ofSetColor(255);
            thresholdedImage.draw(0, 0, 640, 480);
            //            FilteredDepthImage.draw(650, 0, 320, 240);
            //
            ofNoFill();
            ofSetColor(255);
            ofDrawRectangle(kinectROI);
            ofFill();
            
            //           ofSetColor(255);
            //            thresholdedImage.draw(650, 0, 320, 240); // Overwrite depth image
            contourFinder.draw(0, 0);//, 320, 240); // Draw contour finder results
            large.draw();
        } else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP && ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
            ofNoFill();
            ofSetColor(0,255,0,255);
            ofDrawRectangle(kinectROIManualCalib);
            ofFill();
        }
        ofSetColor(255);
    } else if (generalState == GENERAL_STATE_SANDBOX){
        fboProjWindow.draw(0, 0, 640, 480);
        //Draw testing point indicator
        float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 10);
        ofDrawCircle(testPoint.x, testPoint.y, ptSize);
        
        ofRectangle imgROI;
        imgROI.setFromCenter(testPoint, 20, 20);
        kinectColorImage.setROI(imgROI);
        kinectColorImage.drawROI(650, 10, 100, 100);
        ofDrawCircle(700, 60, ptSize);
        
        if (firstImageReady) {
            //            FilteredDepthImage.setROI(imgROI);
            float * roi_ptr = (float*)FilteredDepthImage.getFloatPixelsRef().getData() + ((int)(imgROI.y)*kinectResX) + (int)imgROI.x;
            ofFloatPixels ROIDpt;
            ROIDpt.setNumChannels(1);
            ROIDpt.setFromAlignedPixels(roi_ptr,imgROI.width,imgROI.height,1,kinectResX*4);
            Dptimg.setFromPixels(ROIDpt);
            
            Dptimg.setNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);
            Dptimg.contrastStretch();
            Dptimg.draw(650, 120, 100, 100);
            ofDrawCircle(700, 170, ptSize);
        }
    } else if (generalState == GENERAL_STATE_GAME1) {
        vhcle.draw(0,0);
        contourFinder.draw(0,0);
    }
    
    //    drawGui();
}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){
    // Main draw call for projector window
    fboProjWindow.draw(0, 0);
    
    if (generalState == GENERAL_STATE_GAME1)
        drawVehicles();
}

//--------------------------------------------------------------
void ofApp::drawChessboard(int x, int y, int chessboardSize) {
    // Draw the calibration chess board on the projector window
    float w = chessboardSize / chessboardX;
    float h = chessboardSize / chessboardY;
    
    float xf = x-chessboardSize/2; // x and y are chess board center size
    float yf = y-chessboardSize/2;
    
    currentProjectorPoints.clear();
    
    fboProjWindow.begin();
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
    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::drawTestingPoint(ofVec2f projectedPoint) {
    // Draw a testing point on the projector window
    float ptSize = ofMap(sin(ofGetFrameNum()*0.1), -1, 1, 3, 40);
    fboProjWindow.begin();
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
void ofApp::drawSandbox() {
    // Main sandbox drawing function
    fboProjWindow.begin();
    ofBackground(0);
    FilteredDepthImage.getTexture().bind();
    heightMapShader.begin();
    
    heightMapShader.setUniformMatrix4f("kinectProjMatrix",kinectProjMatrix.getTransposedOf(kinectProjMatrix)); // We transpose matrices since OpenGL is row-major order and OpenFrameworks is column-major order
    heightMapShader.setUniformMatrix4f("kinectWorldMatrix",kinectWorldMatrix.getTransposedOf(kinectWorldMatrix));
    heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
    heightMapShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    heightMapShader.setUniform4f("basePlaneEq", basePlaneEq);
    
    heightMapShader.setUniformTexture("heightColorMapSampler",heightMap.getTexture(), 2);
    heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 3);
    heightMapShader.setUniform1f("contourLineFactor", contourLineFactor);
    heightMapShader.setUniform1i("drawContourLines", drawContourLines);
    
    mesh.draw();
    
    heightMapShader.end();
    FilteredDepthImage.getTexture().unbind();
    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::prepareContourLines()
{
    // Prepare contour line fbo, send to the sandbox shader to draw contour lines
    contourLineFramebufferObject.begin();
    ofClear(255,255,255, 0);
    
    FilteredDepthImage.getTexture().bind();
	elevationShader.begin();
    
    elevationShader.setUniformMatrix4f("kinectProjMatrix",kinectProjMatrix.getTransposedOf(kinectProjMatrix)); // Transpose since OpenGL is row-major order
    elevationShader.setUniformMatrix4f("kinectWorldMatrix",kinectWorldMatrix.getTransposedOf(kinectWorldMatrix));
    elevationShader.setUniform2f("contourLineFboTransformation",ofVec2f(contourLineFboScale,contourLineFboOffset));
    elevationShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    elevationShader.setUniform4f("basePlaneEq", basePlaneEq);
    
    mesh.draw();
    
    elevationShader.end();
    FilteredDepthImage.getTexture().unbind();
    contourLineFramebufferObject.end();
}

//--------------------------------------------------------------
void ofApp::prepareMotherPlateform(){
    //    float back = 0.0f;
    //    if (false){//game){
    //        if (type == 0 || type == 2) {// Test fish
    //            int dx = x-motherFish.x;
    //            int dy = y-motherFish.y;
    //            if (abs(dx)<plateformSize && abs(dy)<plateformSize)
    //                if ((dx*dx+dy*dy)<plateformSize*plateformSize)
    //                    back = motherFish.z;
    //        }
    //        if (type == 1 || type == 2) {// Test fish
    //            int dx = x-motherRabbit.x;
    //            int dy = y-motherRabbit.y;
    //            if (abs(dx)<plateformSize && abs(dy)<plateformSize)
    //                if ((dx*dx+dy*dy)<plateformSize*plateformSize)
    //                    back = motherRabbit.z;
    //        }
    //    }
    //    return back;
}

//--------------------------------------------------------------
void ofApp::drawFlowField()
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
void ofApp::drawArrow(ofVec2f projectedPoint, ofVec2f v1)
{
    // Draw an arrow of dimensions v1 on the projector fbo at location projectedPoint
    fboProjWindow.begin();
    
    ofFill();
    ofPushMatrix();
    
    ofTranslate(projectedPoint);
    
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, v1.x, v1.y);
    ofDrawCircle(v1.x, v1.y, 5);
    
    ofPopMatrix();
    
    fboProjWindow.end();
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
void ofApp::drawGui(){
    
}
//--------------------------------------------------------------
void ofApp::addPointPair() {
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
ofVec2f ofApp::computeTransform(ofVec4f vin) // vin is in kinect world coordinate
{
    ofVec4f screenPos = kinectProjMatrix*vin;
    ofVec2f projectedPoint(screenPos.x/screenPos.z, screenPos.y/screenPos.z);
    return projectedPoint;
}

//--------------------------------------------------------------
ofVec4f ofApp::getWorldCoord(float x, float y) // vin is in kinect image coordinate
{
    ofVec4f wc = ofVec2f(x, y);
    wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)y*kinectResX+(int)x];
    wc.w = 1;
    ofVec4f retVal = kinectWorldMatrix*wc*wc.z;
    retVal.w = 1;
    return retVal;
}

//--------------------------------------------------------------
// Find the normal of a plane inside the kinectROI to take as baseplane
void ofApp::computeBasePlane(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    
    ofLogVerbose("GreatSand") << "computeBasePlane(): smallROI: " << smallROI ;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    ofLogVerbose("GreatSand") << "computeBasePlane(): sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh ;
    
    if (sw*sh == 0) {
        ofLogVerbose("GreatSand") << "computeBasePlane(): smallROI is null, cannot compute base plane normal" ;
        return;
    }
    
    ofVec4f pt;
    
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    ofLogVerbose("GreatSand") << "computeBasePlane(): Computing points in smallROI : " << sw*sh ;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            points[x+y*sw] = ofVec3f(getWorldCoord(x+sl, y+st));
        }
    }
    
    ofLogVerbose("GreatSand") << "computeBasePlane(): Computing plane from points" ;
    basePlaneEq = plane_from_points(points, sw*sh);
    basePlaneNormal = ofVec3f(basePlaneEq);
    basePlaneOffset = ofVec3f(0,0,-basePlaneEq.w);
    
    // Update ranges and planes
    setRangesAndBasePlaneEquation();
}
//--------------------------------------------------------------
// Find the offset of a plane inside the kinectROI to take as maxoffset (over which the depth values are disregarded
void ofApp::findMaxOffset(){
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
void ofApp::autoCalib(){
    if (autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE){
        //        fboProjWindow.begin();
        //        ofBackground(255);
        //        fboProjWindow.end();
        //        updateROIAutoSetup();
        //        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        mainMessage = "Please flatten the sand surface carefully and press [SPACE]. This surface will give the sea level.";
        resultMessage = "Please flatten the sand surface carefully and press [SPACE]. This surface will give the sea level.";
    } else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT){
        if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
            if (kinectROI.getArea() == 0)
            {
                ofLogVerbose("GreatSand") << "autoCalib(): Could not find kinect ROI, stopping autocalib" ;
                autoCalibState = AUTOCALIB_STATE_DONE;
            } else {
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
                resultMessage = "Please put a board on the sandbox and press [SPACE]";
                mainMessage = "Please put a board on the sandbox and press [SPACE]";
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
        }
        autoCalibState = AUTOCALIB_STATE_DONE;
    } else if (autoCalibState == AUTOCALIB_STATE_DONE){
    }
}

//--------------------------------------------------------------
// Find kinect ROI of the sandbox
void ofApp::updateROIAutoSetup(){
    //updateROIFromColorImage();
    updateROIFromDepthImage();
}

//--------------------------------------------------------------
// Find kinect ROI of the sandbox from the colored image
void ofApp::updateROIFromColorImage(){
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
void ofApp::updateROIFromDepthImage(){
    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) {
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
        //  }
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
    }
}

//--------------------------------------------------------------
void ofApp::updateROIManualSetup(){
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
void ofApp::updateKinectGrabberROI(){
    // Send the new mode informations to the kinect grabber
#if __cplusplus>=201103
    kinectgrabber.ROIchannel.send(std::move(kinectROI));
#else
    kinectgrabber.ROIchannel.send(kinectROI);
#endif
}
//--------------------------------------------------------------
void ofApp::updateMode(){
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
void ofApp::keyPressed(int key){
    if (key==' '){
        if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
        {
            ofLogVerbose("GreatSand") << "keyPressed(): Adding point pair" ;
            addPointPair();
        } else if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_AUTOCALIB && autoCalibState == AUTOCALIB_STATE_INIT_FIRST_PLANE)
        {
            autoCalibState = AUTOCALIB_STATE_INIT_POINT;
        } else if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_AUTOCALIB && autoCalibState == AUTOCALIB_STATE_NEXT_POINT) {
            if (!upframe)
                upframe = true;
        } else if (generalState == GENERAL_STATE_GAME1)
        {
            setupVehicles();
        }
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
bool ofApp::loadSettings(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
    xml.setTo("KINECTSETTINGS");
    kinectROI = xml.getValue<ofRectangle>("kinectROI");
    basePlaneNormal = xml.getValue<ofVec3f>("basePlaneNormal");
    basePlaneOffset = xml.getValue<ofVec3f>("basePlaneOffset");
    basePlaneEq = xml.getValue<ofVec4f>("basePlaneEq");
    maxOffset = xml.getValue<float>("maxOffset");
    
    return true;
}

//--------------------------------------------------------------
bool ofApp::saveSettings(string path){
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

//--------------------------------------------------------------
void ofApp::setupGui(){
	// we add this listener before setting up so the initial circle resolution is correct
	circleResolution.addListener(this, &ofApp::circleResolutionChanged);
	ringButton.addListener(this, &ofApp::ringButtonPressed);
    
    // Calibration panel
	calibration.setup("Calibration", "GUI_calibration_panel_settings.xml", 10, 10);
    calibration.add(autocalib.setup("Auto Calibration"));
    calibration.add(manualROI.setup("Manually define sand region"));
    calibration.add(manualcalib.setup("Manually calibrate kinect & projector"));
    calibration.add(calibcheck.setup("Test kinect & projectorcalibration"));

    // Animal panel
    animals.setup("Animals", "GUI_animals_panel_settings.xml", 110, 10);
    animals.add(resetanimallocations.setup("Reset animal locations"));
    animals.add(removeanimals.setup("Remove all animals"));
	animals.add(motherfish.setup("Mother fish", motherFish));
    animals.add(motherrabbit.setup("Mother rabbit", motherRabbit));
	animals.add(fishnumber.setup("Number of fish", 0, 0, 10));
    animals.add(rabbitsnumber.setup("Number of rabbits", 0, 0, 10));
    
    // Sea level panel
	sealevel.setup("Sea level", "GUI_sealevel_panel_settings.xml", 210, 10);
    sealevel.add(resetsealevel.setup("Reset sea level location and tilt"));
    sealevel.add(sealeveltilt.setup("Tilt sea level", ofVec2f(0,0), ofVec2f(-30, -30), ofVec2f(30,30)));
	sealevel.add(sealevelz.setup("Sea level vertical location", 0, heightMap.getScalarRangeMin(), heightMap.getScalarRangeMax()));

    // Display panel
	display.setup("Display", "GUI_display_panel_settings.xml", 310, 10);
    display.add(showcontourlines.setup("Display contourlines", drawContourLines));
    display.add(followbigchanges.setup("Quick reaction (follow hands)", true));
    display.add(spatialfilter.setup("Spatial filtering", true));
    display.add(contourlinesdistance.setup("Contourlines distance", contourLineDistance, 1, 100));
    display.add(highestlevel.setup("Highest detection level", maxOffset, 0, 100));

	colors.setup("Colors", "GUI_colors_panel_settings.xml", 410, 10);
    
    gui.add(filled.setup("fill", true));
	gui.add(radius.setup("radius", 140, 10, 300));
	gui.add(center.setup("center", ofVec2f(ofGetWidth()*.5, ofGetHeight()*.5), ofVec2f(0, 0), ofVec2f(ofGetWidth(), ofGetHeight())));
	gui.add(color.setup("color", ofColor(100, 100, 140), ofColor(0, 0), ofColor(255, 255)));
	gui.add(circleResolution.setup("circle res", 5, 3, 90));
	gui.add(twoCircles.setup("two circles"));
	gui.add(ringButton.setup("ring"));
	gui.add(screenSize.setup("screen size", ofToString(ofGetWidth())+"x"+ofToString(ofGetHeight())));
}


