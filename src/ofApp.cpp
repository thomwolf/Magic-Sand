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
    autoCalibState = AUTOCALIB_STATE_INIT_ROI;
    saved = false;
    loaded = false;
    calibrated = false;
    firstImageReady = false;
    waitingForFirstImage = false;
    
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
    fishNum = 5;
    rabbitsNum = 5;
    motherFishPlatformSize = 20;
    motherRabbitPlatformSize = 20;
    
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
    cout << "Loading shadersES2"<< endl;
	loaded = loaded && elevationShader.load("shadersES2/elevationShader");
	loaded = loaded && heightMapShader.load("shadersES2/heightMapShader");
#else
	if(ofIsGLProgrammableRenderer()){
        cout << "Loading shadersGL3/elevationShader"<< endl;
		loaded = loaded && elevationShader.load("shadersGL3/elevationShader");
        cout << "Loading shadersGL3/heightMapShader"<< endl;
		loaded = loaded && heightMapShader.load("shadersGL3/heightMapShader");
	}else{
        cout << "Loading shadersGL2/elevationShader"<< endl;
		loaded = loaded && elevationShader.load("shadersGL2/elevationShader");
        cout << "Loading shadersGL2/heightMapShader"<< endl;
		loaded = loaded && heightMapShader.load("shadersGL2/heightMapShader");
	}
#endif
    if (!loaded)
    {
        cout << "shader not loaded" << endl;
        exit();
    }
    
    //Try to load calibration file if possible
    if (kpt.loadCalibration("calibration.xml"))
    {
        cout << "Calibration loaded " << endl;
        kinectProjMatrix = kpt.getProjectionMatrix();
        cout << "kinectProjMatrix: " << kinectProjMatrix << endl;
        loaded = true;
        calibrated = true;
        generalState = GENERAL_STATE_GAME1;
        updateMode();
        waitingForFirstImage = true;
    } else {
        cout << "Calibration could not be loaded " << endl;
    }
    
    //Try to load settings file if possible
    if (loadSettings("settings.xml"))
    {
        cout << "Settings loaded " << endl;
        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
    } else {
        cout << "Settings could not be loaded " << endl;
    }
    
	// finish kinectgrabber setup and start the grabber
    kinectgrabber.setupFramefilter(gradFieldresolution, maxOffset, kinectROI);
    kinectWorldMatrix = kinectgrabber.getWorldMatrix();
    cout << "kinectWorldMatrix: " << kinectWorldMatrix << endl;
    
    // Setup elevation ranges and base plane equation
    setRangesAndBasePlaneEquation();
    
    // Setup gradient field
    setupGradientField();
    
    setupVehicles();
    
    setupGui();
    
	kinectgrabber.startThread(true);
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
void ofApp::setupGui(){
    //required call
    gui.setup();

    ImGui::GetIO().MouseDrawCursor = false;
    //backgroundColor is stored as an ImVec4 type but can handle ofColor
    backgroundColor = ofColor(114, 144, 154);
    show_test_window = false;
    show_another_window = true;
    floatValue = basePlaneOffset.z;

    //load your own ofImage
    imageButtonSource.load("of.png");
    imageButtonID = gui.loadImage(imageButtonSource);

    //or have the loading done for you if you don't need the ofImage reference
    //imageButtonID = gui.loadImage("of.png");

    //can also use ofPixels in same manner
    ofLoadImage(pixelsButtonSource, "of_upside_down.png");
    pixelsButtonID = gui.loadPixels(pixelsButtonSource);

    //and alt method
    //pixelsButtonID = gui.loadPixels("of_upside_down.png");

    //pass in your own texture reference if you want to keep it
    textureSourceID = gui.loadTexture(textureSource, "of_upside_down.png");

    //or just pass a path
    //textureSourceID = gui.loadTexture("of_upside_down.png");

    ofLogVerbose() << "textureSourceID: " << textureSourceID;
    doSetTheme = false;
    doThemeColorsWindow = false;
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
    //if(elevationMin<heightMap.getScalarRangeMin())
    basePlaneEq=getPlaneEquation(basePlaneOffset,basePlaneNormal); //homogeneous base plane equation
    //basePlaneEq /= ofVec4f(depthNorm, depthNorm, 1, depthNorm); // Normalized coordinates for the shader (except z since it is already normalized in the Depthframe)
    //update vehicles base plane eq
    //   updateVehiclesBasePlaneEq();
    
    elevationMin=-heightMap.getScalarRangeMin();///depthNorm;
    //if(elevationMax>heightMap.getScalarRangeMax())
    elevationMax=-heightMap.getScalarRangeMax();///depthNorm;
    
    // Calculate the  height map elevation scaling and offset coefficients
	heightMapScale =(heightMap.getNumEntries()-1)/((elevationMax-elevationMin));
	heightMapOffset =0.5/heightMap.getNumEntries()-heightMapScale*elevationMin;
    
    // Set the FilteredDepthImage native scale - converted to 0..1 when send to the shader
    FilteredDepthImage.setNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);//2000/depthNorm);
    //    contourLineFramebufferObject.setNativeScale(elevationMin, elevationMax);
    
    // Calculate the  FilteredDepthImage scaling and offset coefficients
	FilteredDepthScale = elevationMin-elevationMax;
	FilteredDepthOffset = basePlaneOffset.z+elevationMax;
    
    // Calculate the contourline fbo scaling and offset coefficients
	contourLineFboScale = elevationMin-elevationMax;
	contourLineFboOffset = elevationMax;
    contourLineFactor = contourLineFboScale/(contourLineDistance);
    
    cout << "basePlaneOffset: " << basePlaneOffset << endl;
    cout << "basePlaneNormal: " << basePlaneNormal << endl;
    cout << "basePlaneEq: " << basePlaneEq << endl;
    cout << "elevationMin: " << elevationMin << endl;
    cout << "elevationMax: " << elevationMax << endl;
    cout << "heightMap.getNumEntries(): " << heightMap.getNumEntries() << endl;
    cout << "contourLineDistance: " << contourLineDistance << endl;
    cout << "maxOffset: " << maxOffset << endl;
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
    // setup vehicles
    fish.resize(fishNum);
    rabbits.resize(rabbitsNum);
    for (auto & f : fish){
        ofPoint location(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
        f.setup(location.x, location.y, kinectROI);//, kinectWorldMatrix, basePlaneEq, kinectResX, gradFieldcols, gradFieldrows, gradFieldresolution);
        f.animalCoef = 1;
    }
    for (auto & r : rabbits){
        ofPoint location(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
        r.setup(location.x, location.y, kinectROI);//, kinectWorldMatrix, basePlaneEq, kinectResX, gradFieldcols, gradFieldrows, gradFieldresolution);
        r.animalCoef = -1;
    }
    
    if (firstImageReady)
    {
        vhcle = FilteredDepthImage;
        float elevation;
        int earth = 0;
        for (int x = kinectROI.getLeft(); x<kinectROI.getRight(); x ++){ // Project on base plane
            for (int y = kinectROI.getTop(); y < kinectROI.getBottom(); y++){
                ofVec4f wc = ofVec2f(x, y);
                wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[y*kinectResX+x];
                wc.w = 1;
                ofVec4f vertexCc = kinectWorldMatrix*wc*wc.z;
                vertexCc.w = 1;
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
        cout << "Rapport earth/water : "<< rapprt << endl;
        
        //        contourFinder.findContours(vhcle, 50, kinectResX*kinectResY, 20, true);
        //
        //        vector<ofPolyline>  water;
        //        vector<ofRectangle>  bounding;
        //        for (int i = 0; i < contourFinder.nBlobs; i++) {
        //            ofxCvBlob blobContour = contourFinder.blobs[i];
        //            if (blobContour.hole) {
        //                water.push_back(ofPolyline(blobContour.pts));//.getResampledByCount(50);
        //                bounding.push_back(blobContour.boundingRect);
        //            }
        //        }
        ofPoint location;
        
        //        cout << "nBlobs : " << contourFinder.nBlobs << endl;
        //        cout << "water : " << water.size() << endl;
        //
        if (rapprt < 0.8 && rapprt > 0.2){ // Water detection was performed ok
            bool outsidewater = false;
            bool insidewater = false;
            
            for (auto & f : fish){
                insidewater = false;
                while (!insidewater) {
                    location = ofPoint(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
                    if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 0)
                        insidewater = true;
                }
                f.setup(location.x, location.y, kinectROI);//, kinectWorldMatrix, basePlaneEq, kinectResX, gradFieldcols, gradFieldrows, gradFieldresolution);
                f.animalCoef = 1;
            }
            for (auto & r : rabbits){
                outsidewater = false;
                while (!outsidewater) {
                    location = ofPoint(ofRandom(kinectROI.getLeft(),kinectROI.getRight()), ofRandom(kinectROI.getTop(),kinectROI.getBottom()));
                    if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 255)
                        outsidewater = true;
                }
                r.setup(location.x, location.y, kinectROI);//, kinectWorldMatrix, basePlaneEq, kinectResX, gradFieldcols, gradFieldrows, gradFieldresolution);
                r.animalCoef = -1;
            }
            int minborderDist = 40;
            ofRectangle internalBorders = kinectROI;
            internalBorders.scaleFromCenter((kinectROI.width-minborderDist)/kinectROI.width, (kinectROI.height-minborderDist)/kinectROI.height);
            
            ofPoint close;
            outsidewater = false;
            while (!outsidewater) {
                location = ofPoint(ofRandom(internalBorders.getLeft(),internalBorders.getRight()), ofRandom(internalBorders.getTop(),internalBorders.getBottom()));
                if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 255)
                    outsidewater = true;
            }
            motherFish = location; // Put the Fish mother outside of the water to be sure the fish cannot reach her without help
            
            // move mother Fish plateform location under the sea level
            float seaLevelDistance = 10; // We want the plateform not exacty at the sea level but a little bit under
            ofVec4f wc = location;
            wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[((int)location.y*kinectResX)+(int)location.x];
            wc.w = 1;
            ofVec4f vertexCc;
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
            
            insidewater = false;
            while (!insidewater) {
                location = ofPoint(ofRandom(internalBorders.getLeft(),internalBorders.getRight()), ofRandom(internalBorders.getTop(),internalBorders.getBottom()));
                if (vhcle.getPixels().getData()[((int)location.y)*kinectResX+(int)location.x] == 0)
                    insidewater = true;
            }
            motherRabbit = location; // Put the Rabbits mother inside the water to be sure the rabbit cannot reach her without help
            
            // move mother Fish plateform location under the sea level
            seaLevelDistance = 10; // We want the plateform not exacty at the sea level but a little bit under
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
            
            kinectgrabber.framefilter.setGame(motherRabbit, motherFish, 2, motherFishPlatformSize, motherRabbitPlatformSize, true);
            
            // Clean z for distance measurments
            motherRabbit.z = 0;
            motherFish.z = 0;
            
            waitingForFirstImage = false;
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
        if (kinectgrabber.framefilter.firstImageReady)
            firstImageReady = true;
        
        //        //Check values for debug
        //        float maxval = -1000.0;
        //        float minval = 1000.0;
        //        float xf;
        //        for (int i = 0; i<640*480; i ++){
        //            xf = FilteredDepthImage.getFloatPixelsRef().getData()[i];
        //
        //            if (xf > maxval)
        //                maxval = xf;
        //            if (xf < minval)
        //                minval = xf;
        //        }
        //        cout << "FilteredDepthImage maxval : " << maxval << " thresholdedImage minval : " << minval << endl;
        
        // Update grabber stored frame number
		kinectgrabber.lock();
		kinectgrabber.storedframes -= 1;
		kinectgrabber.unlock();
        
        if (generalState == GENERAL_STATE_CALIBRATION) {
            // Get color image from kinect grabber
            ofPixels coloredframe;
            if (kinectgrabber.colored.tryReceive(coloredframe)) {
                kinectColorImage.setFromPixels(coloredframe);
                //            kinectColorImage.updateTexture();
            }
            
            if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
                
                // Get kinect depth image coord
                ofVec2f t = ofVec2f(min((float)kinectResX-1,testPoint.x), min((float)kinectResY-1,testPoint.y));
                ofVec3f worldPoint = ofVec3f(t);
                worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);// / depthNorm;
                ofVec4f wc = ofVec4f(worldPoint);
                wc.w = 1;
                
                ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
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
                    //                       cout << "draw chess" << endl;
                }
            } else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION){
                updateROI();
            } else if (calibrationState == CALIBRATION_STATE_AUTOCALIB){
                autoCalib();
            } else if (calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP){
                if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
                    kinectROIManualCalib.setSize(ofGetMouseX()-kinectROIManualCalib.x,ofGetMouseY()-kinectROIManualCalib.y);
                }
                updateROIManualSetup();
            }
        }
        else if (generalState == GENERAL_STATE_SANDBOX){
            //            // Get kinect depth image coord
            //            ofVec2f t = kinectROI.getCenter();
            //            ofVec3f worldPoint = ofVec3f(t);
            //            worldPoint.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)t.x+kinectResX*(int)t.y];
            //            ofVec4f wc = ofVec4f(worldPoint);
            //            wc.w = 1;
            //
            //            ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
            // drawTestingPoint(projectedPoint);
            
            if (drawContourLines)
                prepareContourLines();
            
            drawSandbox();
        }
        else if (generalState == GENERAL_STATE_GAME1){
            // Get gradient field from kinect grabber
            kinectgrabber.gradient.tryReceive(gradField);
            
            if (drawContourLines)
                prepareContourLines();
            
            drawSandbox();
            
            //drawFlowField();
        }
    }
    if (generalState == GENERAL_STATE_GAME1){
        // update vehicles
        if (firstImageReady) {
            if (waitingForFirstImage)
                setupVehicles();
            // update vehicles datas
            
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
}

//--------------------------------------------------------------
void ofApp::draw(){
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
        kinectColorImage.draw(0, 0, 640, 480);
        FilteredDepthImage.draw(650, 0, 320, 240);
        
        ofNoFill();
        ofSetColor(255);
        ofDrawRectangle(kinectROI);
        ofFill();
        
        ofSetColor(0);
        if (calibrationState == CALIBRATION_STATE_CALIBRATION_TEST){
            ofDrawBitmapStringHighlight("Click on the image to test a point in the RGB image.", 340, 510);
            ofDrawBitmapStringHighlight("The projector should place a green dot on the corresponding point.", 340, 530);
            ofSetColor(255, 0, 0);
            
            //Draw testing point indicator
            float ptSize = ofMap(cos(ofGetFrameNum()*0.1), -1, 1, 3, 40);
            ofDrawCircle(testPoint.x, testPoint.y, ptSize);
            
        } else if (calibrationState == CALIBRATION_STATE_PROJ_KINECT_CALIBRATION)
        {
            
        } else if (calibrationState == CALIBRATION_STATE_ROI_DETERMINATION)
        {
            ofSetColor(255);
            thresholdedImage.draw(650, 0, 320, 240); // Overwrite depth image
            contourFinder.draw(0, 0);//, 320, 240); // Draw contour finder results
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
    
    drawGui();
}

//--------------------------------------------------------------
void ofApp::drawProjWindow(ofEventArgs &args){ // Main draw call for proj window
    //    ofSetColor(ofColor::white);
    fboProjWindow.draw(0, 0);
    
    if (generalState == GENERAL_STATE_GAME1)
        drawVehicles();
}

//--------------------------------------------------------------
void ofApp::drawChessboard(int x, int y, int chessboardSize) { // Prepare proj window fbo
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
void ofApp::drawTestingPoint(ofVec2f projectedPoint) { // Prepare proj window fbo
    float ptSize = ofMap(sin(ofGetFrameNum()*0.1), -1, 1, 3, 40);
    fboProjWindow.begin();
    //    ofBackground(255);
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
    fboProjWindow.begin();
    ofBackground(0);
    //    ofClear(0,0,0,255); // Don't clear the testing point that was previously drawn
    //    ofSetColor(ofColor::red);
    FilteredDepthImage.getTexture().bind();
    heightMapShader.begin();
    
    //    heightMapShader.setUniformTexture("tex1", FilteredDepthImage.getTexture(), 1 ); //"1" means that it is texture 1
    heightMapShader.setUniformMatrix4f("kinectProjMatrix",kinectProjMatrix.getTransposedOf(kinectProjMatrix)); // Transpose since OpenGL is row-major order
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
void ofApp::prepareContourLines() // Prepare contour line fbo
{
	/*********************************************************************
     Prepare the half-pixel-offset frame buffer for subsequent per-fragment
     Marching Squares contour line extraction.
     *********************************************************************/
	
	/* Adjust the projection matrix to render the corners of the final pixels: */
    //	glMatrixMode(GL_PROJECTION);
    //	glPushMatrix();
    //	GLdouble proj[16];
    //	glGetDoublev(GL_PROJECTION_MATRIX,proj);
    //	double xs=double(projResX)/double(projResX+1);
    //	double ys=double(projResY)/double(projResY+1);
    //	for(int j=0;j<4;++j)
    //    {
    //		proj[j*4+0]*=xs;
    //		proj[j*4+1]*=ys;
    //    }
    //	glLoadIdentity();
    //	glMultMatrixd(proj);
	
	/*********************************************************************
     Render the surface's elevation into the half-pixel offset frame
     buffer.
     *********************************************************************/
	
	/* start the elevation shader and contourLineFramebufferObject: */
    
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
	
    //    ofFloatPixels temp;
    //    contourLineFramebufferObject.readToPixels(temp);
    //        //Check values for debug
    //        float maxval = -1000.0;
    //        float minval = 1000.0;
    //        float xf;
    //        for (int i = 0; i<640*480; i ++){
    //            xf = temp.getData()[i];
    //
    //            if (xf > maxval)
    //                maxval = xf;
    //            if (xf < minval)
    //                minval = xf;
    //        }
    //        cout << "contourLineFramebufferObject temp maxval : " << maxval << " temp minval : " << minval << endl;
	/*********************************************************************
     Restore previous OpenGL state.
     *********************************************************************/
	
	/* Restore the original viewport and projection matrix: */
    //	glPopMatrix();
    //	glMatrixMode(GL_MODELVIEW);
    //	glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
    //
    //	/* Restore the original clear color and frame buffer binding: */
    //	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,currentFrameBuffer);
    //	glClearColor(currentClearColor[0],currentClearColor[1],currentClearColor[2],currentClearColor[3]);
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
            // Get kinect depth image coord
            ofVec2f t = ofVec2f((colPos*gradFieldresolution) + gradFieldresolution/2, rowPos*gradFieldresolution  + gradFieldresolution/2);
            ofVec3f worldPoint = ofVec3f(t);
            worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);// / depthNorm;
            ofVec4f wc = ofVec4f(worldPoint);
            wc.w = 1;
            ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
            
            ofVec2f v2 = gradField[colPos + (rowPos * gradFieldcols)];
            
            if (v2.length() > maxsize)
                maxsize = v2.length();
            
            v2 *= arrowLength;
            
            drawArrow(projectedPoint, v2);
        }
    }
    //    cout << "max arrow size : " << maxsize << endl;
}

//--------------------------------------------------------------
void ofApp::drawArrow(ofVec2f projectedPoint, ofVec2f v1)
{
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
        wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)futureLocation.x+kinectResX*(int)futureLocation.y];
        wc.w = 1;
        ofVec4f vertexCc = kinectWorldMatrix*wc*wc.z;
        vertexCc.w = 1;
        float elevation = -basePlaneEq.dot(vertexCc);
        if (v.animalCoef*elevation > 0)
        {
            water = false;
            beachDist = i;
            beachSlope = -v.animalCoef*gradField[(int)(futureLocation.x/gradFieldresolution)+gradFieldcols*((int)(futureLocation.y/gradFieldresolution))];
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
    v.updateBeachDetection(!water, beachDist, beachSlope);
    //        v.updateFutureElevationAndGradient(futureZ, futureGradient);
}

//--------------------------------------------------------------
//void ofApp::updateVehiclesBasePlaneEq(){
//    for (auto & v : vehicles){
//        v.updateBasePlaneEq(basePlaneEq);
//    }
//}

//--------------------------------------------------------------
void ofApp::drawVehicles()
{
    drawMotherFish();
    drawMotherRabbit();
    for (auto & f : fish){
        drawFish(f);//, forces);
    }
    for (auto & r : rabbits){
        drawRabbit(r);//, forces);
    }
}

//--------------------------------------------------------------
void ofApp::drawFish(Fish& v)//, std::vector<ofVec2f> forces)
{
    // Get vehicule coord
    ofVec2f t = v.getLocation();
    
    ofVec3f worldPoint = ofVec3f(t);
    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    ofVec4f wc = ofVec4f(worldPoint);
    wc.w = 1;
    ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    
    //        ofVec2f force = v.getCurrentForce();
    ofVec2f velocity = v.getVelocity();
    //        std::vector<ofVec2f> forces = v.getForces();
    float angle = v.getAngle(); // angle of the fish
    
    //    fboProjWindow.begin();
    // Tail position
    float nv = 0.5;//velocity.lengthSquared()/10; // Tail movement amplitude
    float fact = 50+250*velocity.length()/v.topSpeed;
    float tailangle = nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    //float tailangle = ofMap(sin(ofGetFrameNum()*10), -1, 1, -nv, nv);
    
    // Color of the fish
    nv = 255;
    fact = 50;
    float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
    
    ofPushMatrix();
    
    ofTranslate(projectedPoint);
    
    //    ofSetColor(255);
    ofVec2f frc = ofVec2f(100, 0);
    //    frc.rotate(angle);
    //    ofDrawLine(0, 0, frc.x, frc.y);
    //    ofDrawRectangle(frc.x, frc.y, 5, 5);
    //
    //    for (int i=0; i<forces.size(); i++){
    //        ofColor c = ofColor(0); // c is black
    //        float hsb = ((float)i)/((float)forces.size()-1.0)*255.0;
    //        c.setHsb((int)hsb, 255, 255); // rainbow
    //
    //        frc = forces[i];
    //        frc.normalize();
    //        frc *= 100;
    //
    //        ofSetColor(c);
    //        ofDrawLine(0, 0, frc.x, frc.y);
    //        ofDrawCircle(frc.x, frc.y, 5);
    //    }
    //
    ofRotate(angle);
    float sc = 7; // Fish scale
    
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    
    ofPath fish;
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
    
    ofColor c = ofColor(255);
    ofSetColor(c);
    if (v.foundMother())
    {
        //        float hsb = ((float)i)/((float)forces.size()-1.0)*255.0;
        c.setHsb((int)hsb, 255, 255); // rainbow
        ofFill();
        fish.setFillColor(c);
    } else {
        ofNoFill();
    }
    fish.draw();
    
    if (v.foundMother())
    {
        //        float hsb = ((float)i)/((float)forces.size()-1.0)*255.0;
        c.setHsb(255-(int)hsb, 255, 255); // rainbow
        ofSetColor(c);
    }
    ofDrawCircle(0, 0, sc*0.5);
    
    ofPopMatrix();
    
    ofNoFill();
    // Display tail movement indication
    //    t = ofPoint(kinectResX/2, kinectResY/2);
    //    ofPoint front = velocity;
    //    front.normalize();
    //    front *= v.wanderD;
    //    ofPoint circleloc = t;
    //
    //    worldPoint = ofVec3f(circleloc);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofDrawCircle(0, 0, v.wanderR);
    //    ofPopMatrix();
    //
    //	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    //    ofPoint circleOffSet = ofPoint(v.wanderR*cos(tailangle),v.wanderR*sin(tailangle));
    //    ofPoint target = circleloc + circleOffSet;
    //
    //    worldPoint = ofVec3f(target);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofSetColor(255, 0, 0, 255);
    //    ofDrawCircle(0, 0, 5);
    //    ofPopMatrix();
    
    // Display wandering circle
    //    ofPoint front = velocity;
    //    front.normalize();
    //    front *= v.wanderD;
    //    ofPoint circleloc = t + front;
    //
    //    worldPoint = ofVec3f(circleloc);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofDrawCircle(0, 0, v.wanderR);
    //    ofPopMatrix();
    //
    //	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    //    ofPoint circleOffSet = ofPoint(v.wanderR*cos(v.wandertheta+h),v.wanderR*sin(v.wandertheta+h));
    //    ofPoint target = circleloc + circleOffSet;
    //
    //    worldPoint = ofVec3f(target);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofSetColor(255, 0, 0, 255);
    //    ofDrawCircle(0, 0, 5);
    //    ofPopMatrix();
    
    
    //    fboProjWindow.end();
}

//--------------------------------------------------------------
void ofApp::drawRabbit(Rabbit& v)//, std::vector<ofVec2f> forces)
{
    // Get vehicule coord
    ofVec2f t = v.getLocation();
    
    ofVec3f worldPoint = ofVec3f(t);
    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    ofVec4f wc = ofVec4f(worldPoint);
    wc.w = 1;
    ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    
    //        ofVec2f force = v.getCurrentForce();
    ofVec2f velocity = v.getVelocity();
    //        std::vector<ofVec2f> forces = v.getForces();
    float angle = v.getAngle(); // angle of the fish
//    float fact = P+P/2*velocity.length()/3;
//    
//    //    fboProjWindow.begin();
//    float nv = 1;//velocity.lengthSquared()/10; // Tail movement amplitude
//    float footpos = nv/25 * (abs(((int)(ofGetElapsedTimef()*fact*2) % 100) - 50)-25); // Footpos varies between -1 and 1 over time
    
    ofPushMatrix();
    
    ofTranslate(projectedPoint);
    
    //    ofSetColor(255);
    ofVec2f frc = ofVec2f(100, 0);
    //    frc.rotate(angle);
    //    ofDrawLine(0, 0, frc.x, frc.y);
    //    ofDrawRectangle(frc.x, frc.y, 5, 5);
    //
    //    for (int i=0; i<forces.size(); i++){
    //        ofColor c = ofColor(0); // c is black
    //        float hsb = ((float)i)/((float)forces.size()-1.0)*255.0;
    //        c.setHsb((int)hsb, 255, 255); // rainbow
    //
    //        frc = forces[i];
    //        frc.normalize();
    //        frc *= 100;
    //
    //        ofSetColor(c);
    //        ofDrawLine(0, 0, frc.x, frc.y);
    //        ofDrawCircle(frc.x, frc.y, 5);
    //    }
    //
    ofRotate(angle);
    
    float sc = 1; // Rabbit scale
    
    ofFill();
    ofSetLineWidth(1.0);  // Line widths apply to polylines
    
    ofColor c1 = ofColor(255);
    ofColor c2 = ofColor(0);
    if (v.foundMother())
    {
        float nv = 255;
        int fact = 50;
        float et = ofGetElapsedTimef();
        float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
        //        float hsb = ((float)i)/((float)forces.size()-1.0)*255.0;
        c1.setHsb((int)hsb, 255, 255); // rainbow
        c2.setHsb(255-(int)hsb, 255, 255);
    }

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
    
    ofSetColor(c1);
    
    body.setFillColor(c1);
    body.draw();

    ofSetColor(c2);
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
    
    ofSetColor(c2);
    head.setFillColor(c2);
    head.draw();

    ofSetColor(c1);
    ofDrawCircle(8.5*sc, 0, 1*sc);
    
    //    ofPolyline foot;
    //    foot.curveTo( ofPoint(6*sc, -3.5*sc));
    //    foot.curveTo( ofPoint(6*sc, -3.5*sc));
    //    foot.curveTo( ofPoint((10+footpos*4)*sc, -4.5*sc));
    //    foot.curveTo( ofPoint((7+footpos*1)*sc, (-5.75-footpos*0.25)*sc));
    //    foot.curveTo( ofPoint((2+footpos*0.5)*sc, -5*sc));
    //    foot.curveTo( ofPoint((2+footpos*0.5)*sc, -5*sc));
    //    //    body.close();
    //    foot.draw();
    //
    //    foot.clear();
    //    foot.curveTo( ofPoint(6*sc, 3.5*sc));
    //    foot.curveTo( ofPoint(6*sc, 3.5*sc));
    //    foot.curveTo( ofPoint((10+footpos*4)*sc, 4.5*sc));
    //    foot.curveTo( ofPoint((7+footpos*1)*sc, (5.75+footpos*0.25)*sc));
    //    foot.curveTo( ofPoint((2+footpos*0.5)*sc, 5*sc));
    //    foot.curveTo( ofPoint((2+footpos*0.5)*sc, 5*sc));
    //    //    body.close();
    //    foot.draw();
    //
    //    foot.clear();
    //    foot.curveTo( ofPoint(-13*sc, -6*sc));
    //    foot.curveTo( ofPoint(-13*sc, -6*sc));
    //    foot.curveTo( ofPoint((-16-footpos*1)*sc, (-6.75-footpos*0.25)*sc));
    //    foot.curveTo( ofPoint((-19-footpos*3)*sc, -5.5*sc));
    //    foot.curveTo( ofPoint((-16.5-footpos*1.5)*sc, -4*sc));
    //    foot.curveTo( ofPoint(-15*sc, -4*sc));
    //    foot.curveTo( ofPoint(-15*sc, -4*sc));
    //    //    body.close();
    //    foot.draw();
    //
    //    foot.clear();
    //    foot.curveTo( ofPoint(-13*sc, 6*sc));
    //    foot.curveTo( ofPoint(-13*sc, 6*sc));
    //    foot.curveTo( ofPoint((-16-footpos*1)*sc, (6.75+footpos*0.25)*sc));
    //    foot.curveTo( ofPoint((-19-footpos*3)*sc, 5.5*sc));
    //    foot.curveTo( ofPoint((-16.5-footpos*1.5)*sc, 4*sc));
    //    foot.curveTo( ofPoint(-15*sc, 4*sc));
    //    foot.curveTo( ofPoint(-15*sc, 4*sc));
    //    //    body.close();
    //    foot.draw();
    
    
    ofPopMatrix();
    
    // Display wandering circle
    //    ofPoint front = velocity;
    //    front.normalize();
    //    front *= v.wanderD;
    //    ofPoint circleloc = t + front;
    //
    //    worldPoint = ofVec3f(circleloc);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofDrawCircle(0, 0, v.wanderR);
    //    ofPopMatrix();
    //
    //	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    //    ofPoint circleOffSet = ofPoint(v.wanderR*cos(v.wandertheta+h),v.wanderR*sin(v.wandertheta+h));
    //    ofPoint target = circleloc + circleOffSet;
    //
    //    worldPoint = ofVec3f(target);
    //    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    //    wc = ofVec4f(worldPoint);
    //    wc.w = 1;
    //    projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    //
    //    ofPushMatrix();
    //    ofTranslate(projectedPoint);
    //    ofSetColor(255, 0, 0, 255);
    //    ofDrawCircle(0, 0, 5);
    //    ofPopMatrix();
    ofSetColor(255);
    
    //    fboProjWindow.end();
}
//--------------------------------------------------------------
void ofApp::drawMotherFish()//, std::vector<ofVec2f> forces)
{
    // Get vehicule coord
    ofVec2f t = motherFish;
    
    ofVec3f worldPoint = ofVec3f(t);
    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    ofVec4f wc = ofVec4f(worldPoint);
    wc.w = 1;
    ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    
    //    float nv = 0.5;//velocity.lengthSquared()/10; // Tail movement amplitude
    float tailangle = 0;//nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    
    //    ofRotate(angle);
    
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
    
    ofVec3f worldPoint = ofVec3f(t);
    worldPoint.z = kinectgrabber.kinect.getDistanceAt(t.x, t.y);
    ofVec4f wc = ofVec4f(worldPoint);
    wc.w = 1;
    ofVec2f projectedPoint = computeTransform(wc);//kpt.getProjectedPoint(worldPoint);
    
    //    float nv = 1;//velocity.lengthSquared()/10; // Tail movement amplitude
    //    float footpos = nv/25 * (abs(((int)(ofGetElapsedTimef()*fact*2) % 100) - 50)-25); // Footpos varies between -1 and 1 over time
    
    ofPushMatrix();
    
    //    ofRotate(angle);
    
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
    
    //backgroundColor is stored as an ImVec4 type but is converted to ofColor automatically
    
    ofSetBackgroundColor(backgroundColor);
    
    //required to call this at beginning
    gui.begin();
    
    //In between gui.begin() and gui.end() you can use ImGui as you would anywhere else
    
    // 1. Show a simple window
    {
        ImGui::Text("Hello, world!");
        // Adjust display_format to decorate the value with a prefix or a suffix.
        //   "%.3f"         1.234
        //   "%5.2f secs"   01.23 secs
        //   "Gold: %.0f"   Gold: 1
//        bool ImGui::SliderFloat(const char* label, float* v, float v_min, float v_max, const char* display_format, float power)
        ImGui::SliderFloat("Base plane height", &basePlaneOffset.z, 500, 2000, "%.2f mm from the kinect sensor");
        
        //this will change the app background color
        ImGui::ColorEdit3("Background Color", (float*)&backgroundColor);
        if(ImGui::Button("Test Window"))
        {
            show_test_window = !show_test_window;
        }
        
        if (ImGui::Button("Another Window"))
        {
            //bitwise OR
            show_another_window ^= 1;
            
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    }
    // 2. Show another window, this time using an explicit ImGui::Begin and ImGui::End
    if (show_another_window)
    {
        //note: ofVec2f and ImVec2f are interchangeable
        ImGui::SetNextWindowSize(ofVec2f(200,100), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Another Window", &show_another_window);
        ImGui::Text("Hello");
        ImGui::End();
    }
    
    // 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
    if (show_test_window)
    {
        ImGui::SetNextWindowPos(ofVec2f(650, 20), ImGuiSetCond_FirstUseEver);
        ImGui::ShowTestWindow(&show_test_window);
    }
    
    
    bool pressed = ImGui::ImageButton((ImTextureID)(uintptr_t)imageButtonID, ImVec2(200, 200));
    pressed = ImGui::ImageButton((ImTextureID)(uintptr_t)pixelsButtonID, ImVec2(200, 200));
    pressed = ImGui::ImageButton((ImTextureID)(uintptr_t)textureSourceID, ImVec2(200, 200));
    
    
    if(doThemeColorsWindow)
    {
        gui.openThemeColorWindow();
        
    }
    
    //required to call this at end
    gui.end();
    
    if(textureSource.isAllocated())
    {
        //textureSource.draw(ofRandom(200), ofRandom(200));
    }
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
            worldPoint.z = worldPoint.z;
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
ofVec2f ofApp::computeTransform(ofVec4f vin) // vin is in kinect image depth coordinate with normalized z
{
    /* Transform the vertex from depth image space to world space: */
    //    ofVec3f vertexCcxx = kinectgrabber.kinect.getWorldCoordinateAt(vertexDic.x, vertexDic.y, vertexDic.z);
    ofVec4f vertexCc = kinectWorldMatrix*vin*vin.z;
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
// Find normal of the plane inside the kinectROI
void ofApp::computeBasePlane(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    
    cout << "smallROI: " << smallROI << endl;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    cout << "sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh << endl;
    
    if (sw*sh == 0) {
        cout << "smallROI is null, cannot compute base plane normal" << endl;
        return;
    }
    
    ofVec4f pt;
    
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    cout << "Computing points in smallROI : " << sw*sh << endl;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            
            // Get kinect depth image coord
            pt = ofVec4f(x+sl, y+st, 0, 1);
            pt.z = FilteredDepthImage.getFloatPixelsRef().getData()[x+sl + (y+st)*kinectResX];
            
            ofVec3f vertexCc = ofVec3f(kinectWorldMatrix*pt*pt.z); // Translate kinect coord in world coord
            
            points[x+y*sw] = vertexCc;
        }
    }
    
    cout << "Computing plane from points" << endl;
    basePlaneEq = plane_from_points(points, sw*sh);
    basePlaneNormal = ofVec3f(basePlaneEq);
    basePlaneOffset = ofVec3f(0,0,-basePlaneEq.w);
    
    setRangesAndBasePlaneEquation();
}
//--------------------------------------------------------------
// Find normal of the plane inside the kinectROI
void ofApp::findMaxOffset(){
    ofRectangle smallROI = kinectROI;
    smallROI.scaleFromCenter(0.75); // Reduce ROI to avoid problems with borders
    
    cout << "smallROI: " << smallROI << endl;
    int sw = (int) smallROI.width;
    int sh = (int) smallROI.height;
    int sl = (int) smallROI.getLeft();
    int st = (int) smallROI.getTop();
    cout << "sw: " << sw << " sh : " << sh << " sl : " << sl << " st : " << st << " sw*sh : " << sw*sh << endl;
    
    if (sw*sh == 0) {
        cout << "smallROI is null, cannot compute base plane normal" << endl;
        return;
    }
    
    ofVec4f pt;
    
    ofVec3f* points;
    points = new ofVec3f[sw*sh];
    cout << "Computing points in smallROI : " << sw*sh << endl;
    for (int x = 0; x<sw; x++){
        for (int y = 0; y<sh; y ++){
            
            // Get kinect depth image coord
            pt = ofVec4f(x+sl, y+st, 0, 1);
            pt.z = FilteredDepthImage.getFloatPixelsRef().getData()[x+sl + (y+st)*kinectResX];
            
            ofVec3f vertexCc = ofVec3f(kinectWorldMatrix*pt*pt.z); // Translate kinect coord in world coord
            
            points[x+y*sw] = vertexCc;
        }
    }
    
    cout << "Computing plane from points" << endl;
    ofVec4f eqoff = plane_from_points(points, sw*sh);
    maxOffset = -eqoff.w-maxOffsetSafeRange;
    
    cout << "maxOffset" << maxOffset << endl;
    kinectgrabber.setMaxOffset(maxOffset);
}

//--------------------------------------------------------------
// Autoclibration of the sandbox
void ofApp::autoCalib(){
    if (autoCalibState == AUTOCALIB_STATE_INIT_ROI){
        //        fboProjWindow.begin();
        //        ofBackground(255);
        //        fboProjWindow.end();
        updateROI();
        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
    } else if (autoCalibState == AUTOCALIB_STATE_INIT_POINT){
        if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
            if (kinectROI.getArea() == 0)
            {
                cout << "Could not find kinect ROI, stopping autocalib" << endl;
                autoCalibState = AUTOCALIB_STATE_DONE;
            } else {
                computeBasePlane(); // Find base plane
                
                autoCalibPts = new ofPoint[10];
                float cs = 2*chessboardSize/3;
                float css = 3*chessboardSize/4;
                ofPoint sc = ofPoint(projResX/2,projResY/2);
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
            updateROI();
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
                    cout << "Chessboard found for point :" << currentCalibPts << endl;
                    cout << "Adding point pair" << endl;
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
                    cout << "Clear screen found, drawing next chessboard" << endl;
                    cleared = true; // The cleared fbo screen was seen by the kinect
                    ofPoint dispPt = ofPoint(projResX/2,projResY/2)+autoCalibPts[currentCalibPts]; // Compute next chessboard position
                    drawChessboard(dispPt.x, dispPt.y, chessboardSize); // We can now draw the next chess board
                } else {
                    trials++;
                    cout << "Chessboard not found on trial : " << trials << endl;
                    if (trials >10) {
                        cout << "Chessboard could not be found moving chessboard closer to center " << endl;
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
            }
        }
    } else if (autoCalibState == AUTOCALIB_STATE_COMPUTE){
        if (pairsKinect.size() == 0) {
            cout << "Error: No points acquired !!" << endl;
        } else {
            cout << "Calibrating" << endl;
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
void ofApp::updateROI(){
    fboProjWindow.begin();
    ofBackground(255);
    fboProjWindow.end();
    if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT) { // set kinect to max depth range
        ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
        large = ofPolyline();
        threshold = 90;
        
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
        while (threshold < 255){
            //            cout << "Increasing threshold : " << threshold << endl;
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
            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_BINARY_INV);
            //        cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), 1, 255, CV_THRESH_BINARY);
            
            //setFromPixels(kinectColorImage.getPixels());//kinectColorImage.getPixels());
            //            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_TOZERO_INV);
            //            cvThreshold(thresholdedImage.getCvImage(), thresholdedImage.getCvImage(), threshold, 255, CV_THRESH_BINARY);
            
            contourFinder.findContours(thresholdedImage, 12, 640*480, 5, true);
            //contourFinder.findContours(thresholdedImage);
            //ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
            
            ofPolyline small = ofPolyline();
            for (int i = 0; i < contourFinder.nBlobs; i++) {
                ofxCvBlob blobContour = contourFinder.blobs[i];
                if (blobContour.hole) {
                    ofPolyline poly = ofPolyline(blobContour.pts);//.getResampledByCount(50);
                    
                    if (poly.inside(kinectResX/2, kinectResY/2))
                    {
                        //                        cout << "We found a contour lines surroundings the center of the screen" << endl;
                        if (small.size() == 0 || poly.getArea() < small.getArea()) {
                            //                            cout << "We take the smallest contour line surroundings the center of the screen at a given threshold level" << endl;
                            small = poly;
                        }
                    }
                }
            }
            cout << "small.getArea(): " << small.getArea() << endl;
            cout << "large.getArea(): " << large.getArea() << endl;
            if (large.getArea() < small.getArea())
            {
                cout << "We take the largest contour line surroundings the center of the screen at all threshold level" << endl;
                large = small;
            }
            threshold+=1;
        }
        //        if (threshold > 255) {
        kinectROI = large.getBoundingBox();
        kinectROI.standardize();
        cout << "kinectROI : " << kinectROI << endl;
        // We are finished, set back kinect depth range and update ROI
        ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
        kinectgrabber.setKinectROI(kinectROI);
        //update the mesh
        setupMesh();
        //        }
    } else if (ROICalibrationState == ROI_CALIBRATION_STATE_DONE){
    }
}

//--------------------------------------------------------------
void ofApp::updateROIManualSetup(){
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
void ofApp::updateMode(){
    if (generalState == GENERAL_STATE_CALIBRATION)
        ofBackground(255);
    else
        ofBackground(0);
    
    cout << "General state: " << generalState << endl;
    cout << "Calibration state: " << calibrationState << endl;
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
            cout << "Adding point pair" << endl;
            addPointPair();
        }
        if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_AUTOCALIB)
        {
            if (!upframe)
                upframe = true;
        }
        if (generalState == GENERAL_STATE_GAME1)
        {
            setupVehicles();
        }
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
        cout << "maxOffset" << maxOffset << endl;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if (key=='x') {
        maxOffset -= 0.5;
        cout << "maxOffset" << maxOffset << endl;
        kinectgrabber.setMaxOffset(maxOffset);
    } else if (key=='d') {
        doThemeColorsWindow = !doThemeColorsWindow;
        cout << "doThemeColorsWindow: " << doThemeColorsWindow << endl;
    } else if (key=='f') {
        doSetTheme = !doSetTheme;
        cout << "doSetTheme: " << doSetTheme << endl;
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
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        cout << "Finding ROI" << endl;
    } else if (key=='m') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_ROI_MANUAL_SETUP;
        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
        cout << "Updating ROI Manually" << endl;
    } else if (key=='y') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_AUTOCALIB;
        autoCalibState = AUTOCALIB_STATE_INIT_ROI;
        cout << "Starting autocalib" << endl;
    } else if (key=='t') {
        generalState = GENERAL_STATE_CALIBRATION;
        calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
    } else if (key=='b') {
        generalState = GENERAL_STATE_SANDBOX;
    }else if (key=='e') {
        waitingForFirstImage = true;
        generalState = GENERAL_STATE_GAME1;
    } else if (key=='v') {
        if (kpt.saveCalibration("calibration.xml"))
        {
            cout << "Calibration saved " << endl;
            saved = true;
        } else {
            cout << "Calibration could not be saved " << endl;
        }
    } else if (key=='l') {
        if (kpt.loadCalibration("calibration.xml"))
        {
            cout << "Calibration loaded " << endl;
            kinectProjMatrix = kpt.getProjectionMatrix();
            loaded = true;
            calibrated = true;
        } else {
            cout << "Calibration could not be loaded " << endl;
        }
    } else if (key=='j') {
        if (saveSettings("settings.xml"))
        {
            cout << "Settings saved " << endl;
        } else {
            cout << "Settings could not be saved " << endl;
        }
    } else if (key=='k') {
        if (loadSettings("settings.xml"))
        {
            cout << "Settings loaded " << endl;
        } else {
            cout << "Settings could not be loaded " << endl;
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
    if (generalState == GENERAL_STATE_SANDBOX || (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_CALIBRATION_TEST)) {
        if (x<kinectResX && y <kinectResY) {
            testPoint.set(min(x, kinectResX-1), min(y, kinectResY-1));
            
            int idx = (int)testPoint.x+kinectResX*(int)testPoint.y;
            cout << "Depth value at point: " << FilteredDepthImage.getFloatPixelsRef().getData()[idx]<< endl;
            float* sPtr=kinectgrabber.framefilter.statBuffer+3*idx;
            cout << " Number of valid samples statBuffer[0]: " << sPtr[0] << endl;
            cout << " Sum of valid samples statBuffer[1]: " << sPtr[1] << endl; //
            cout << " Sum of squares of valid samples statBuffer[2]: " << sPtr[2] << endl; // Sum of squares of valid samples<< endl;
        } else if (x > 650 && x < 750 && y > 120 && y < 220) {
            ofVec2f tmp = testPoint;
            testPoint.set(tmp.x+(x-700)/5, tmp.y+(y-170)/5);
            
            int idx = (int)testPoint.x+kinectResX*(int)testPoint.y;
            cout << "Depth value at point: " << FilteredDepthImage.getFloatPixelsRef().getData()[idx]<< endl;
            float* sPtr=kinectgrabber.framefilter.statBuffer+3*idx;
            cout << " Number of valid samples statBuffer[0]: " << sPtr[0] << endl;
            cout << " Sum of valid samples statBuffer[1]: " << sPtr[1] << endl; //
            cout << " Sum of squares of valid samples statBuffer[2]: " << sPtr[2] << endl; // Sum of squares of valid samples<< endl;
        }
    }
    if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP){
        if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT)
        {
            kinectROIManualCalib.setPosition(x, y);
            ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
        } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
            kinectROIManualCalib.setSize(x-kinectROIManualCalib.x, y-kinectROIManualCalib.y);
            ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
            
            kinectROI = kinectROIManualCalib;
            kinectROI.standardize();
            cout << "kinectROI : " << kinectROI << endl;
            kinectgrabber.setKinectROI(kinectROI);
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
    
    setRangesAndBasePlaneEquation();
    kinectgrabber.setKinectROI(kinectROI);
    
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
