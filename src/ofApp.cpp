#include "ofApp.h"

using namespace ofxCv;
using namespace cv;
using namespace ofxCSG;

void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
    //    ofSetBackgroundAuto(false); // Avoid autoclear on draw
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    
    // Get projector size
    projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
    
    // Setup kinectProjector
    kinectProjector = std::make_shared<KinectProjector>();
    kinectProjector->setup(projRes);
    
    // Retrieve variables
    kinectRes = kinectProjector->getKinectRes();
    kinectROI = kinectProjector->getKinectROI();
    
    sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector);
    sandSurfaceRenderer->setup(projRes);
    
    fboProjWindow.allocate(projRes.x, projRes.y, GL_RGBA);
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();
    
    setupGui();

    // Vehicles
    fishNum = 1;
    rabbitsNum = 0;
    showMotherFish = false;
    showMotherRabbit = false;
    motherPlatformSize = 20;
    waitingToInitialiseVehicles = true;
}

void ofApp::setupVehicles(){
    for (int i=0; i< fishNum; i++){
        addNewFish();
    }
    for (int i=0; i< rabbitsNum; i++){
        addNewRabbit();
    }
    if (showMotherFish)
        addMotherFish();
    if (showMotherRabbit)
        addMotherRabbit();
    
    waitingToInitialiseVehicles = false;
}

void ofApp::addNewFish(){
    ofVec2f location;
    location = findRandomVehicleLocation(kinectROI, true);
    auto f = Fish(kinectProjector, location, kinectROI, true);
    f.setup();
    fish.push_back(f);
}

void ofApp::addNewRabbit(){
    ofVec2f location;
    location = findRandomVehicleLocation(kinectROI, false);
    auto r = Rabbit(kinectProjector, location, kinectROI, false);
    r.setup();
    rabbits.push_back(r);
}

void ofApp::addMotherFish(){
    int minborderDist = 40;
    ofRectangle internalBorders = kinectROI;
    internalBorders.scaleFromCenter((kinectROI.width-minborderDist)/kinectROI.width, (kinectROI.height-minborderDist)/kinectROI.height);

    // Find a location for the Fish mother outside of the water to be sure the fish cannot reach her without help
    motherFish = findRandomVehicleLocation(internalBorders, false);
    
    // Set the mother Fish plateform location under the sea level
    motherFish.z = kinectProjector->elevationToKinectDepth(-10, motherFish.x, motherFish.y);
}

void ofApp::addMotherRabbit(){
    int minborderDist = 40;
    ofRectangle internalBorders = kinectROI;
    internalBorders.scaleFromCenter((kinectROI.width-minborderDist)/kinectROI.width, (kinectROI.height-minborderDist)/kinectROI.height);
    
    // Find a location for the Rabbits mother inside of the water to be sure the rabbits cannot reach her without help
    motherRabbit = findRandomVehicleLocation(internalBorders, true);
    
    // Set the mother Rabbit plateform location over the sea level
    motherRabbit.z = kinectProjector->elevationToKinectDepth(10, motherRabbit.x, motherRabbit.y);
}

ofVec2f ofApp::findRandomVehicleLocation(ofRectangle area, bool liveInWater){
    ofVec2f location;
    bool okwater = false;
    while (!okwater) {
        float x = ofRandom(area.getLeft(),area.getRight());
        float y = ofRandom(area.getTop(),area.getBottom());
        bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater))
            okwater = true;
    }
    return location;
}

void ofApp::update(){
    kinectProjector->update();
    sandSurfaceRenderer->update();
    if (kinectProjector->isImageStabilized()) {
        if (waitingToInitialiseVehicles)
            setupVehicles();
        for (auto & f : fish){
            f.applyBehaviours(motherFish);
            f.update();
        }
        for (auto & r : rabbits){
            r.applyBehaviours(motherRabbit);
            r.update();
        }
    }
    
    if (kinectProjector->isROIUpdated())
        sandSurfaceRenderer->setupMesh();
    if (kinectProjector->isBasePlaneUpdated())
        sandSurfaceRenderer->updateRangesAndBasePlane();
}

void ofApp::draw(){
    if (kinectProjector->isCalibrating()){
        kinectProjector->drawMainWindow();
    } else {
        sandSurfaceRenderer->draw();
        drawVehicles();
    }
//    fboProjWindow.draw(0, 0, 640, 480);
//    ofNoFill();
//    ofSetColor(255);
//    ofDrawRectangle(kinectROI);
//    ofFill();
}

void ofApp::drawProjWindow(ofEventArgs &args){
    if (kinectProjector->isCalibrating()){
        kinectProjector->drawProjectorWindow();
    } else {
        sandSurfaceRenderer->draw();
        drawVehicles();
    }
}

void ofApp::drawVehicles()
{
    for (auto & f : fish){
        f.draw();
    }
    for (auto & r : rabbits){
        r.draw();
    }
    if (showMotherFish)
        drawMotherFish();
    if (showMotherRabbit)
        drawMotherRabbit();
}

void ofApp::drawMotherFish()
{
    // Mother fish scale
    float sc = 10;
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    float tailangle = 0;//nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    
    ofNoFill();
    ofPushMatrix();
    ofTranslate(kinectProjector->kinectCoordToProjCoord(motherFish.x+tailSize, motherFish.y));
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
    ofSetLineWidth(2.0);
    fish.draw();
    ofSetColor(255);
    ofDrawCircle(0, 0, 5);
    ofPopMatrix();
}

void ofApp::drawMotherRabbit()
{
    float sc = 2; // MotherRabbit scale
    ofPushMatrix();
    ofTranslate(kinectProjector->kinectCoordToProjCoord(motherRabbit.x+5*sc, motherRabbit.y));
    ofFill();
    ofSetLineWidth(1.0);
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
void ofApp::keyPressed(int key){
    if (key==' '){
            tIndex = tIndex < themes.size()-1 ? tIndex+1 : 0;
            gui->setTheme(themes[tIndex]);
    }
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
//    } else if (key=='a') {
//        chessboardSize -= 20;
//    } else if (key=='z') {
//        chessboardSize += 20;
//    } else if (key=='q') {
//        basePlaneOffset.z += 0.5;
//        setRangesAndBasePlaneEquation();
//    } else if (key=='s') {
//        basePlaneOffset.z -= 0.5;
//        setRangesAndBasePlaneEquation();
//    } else if (key=='g') {
//        contourLineDistance += 1.0;
//        setRangesAndBasePlaneEquation();
//    } else if (key=='h') {
//        contourLineDistance -= 1.0;
//        setRangesAndBasePlaneEquation();
//    } else if (key=='w') {
//        maxOffset += 0.5;
//        ofLogVerbose("GreatSand") << "keyPressed(): maxOffset" << maxOffset ;
//        kinectgrabber.setMaxOffset(maxOffset);
//    } else if (key=='x') {
//        maxOffset -= 0.5;
//        ofLogVerbose("GreatSand") << "keyPressed(): maxOffset" << maxOffset ;
//        kinectgrabber.setMaxOffset(maxOffset);
//    } else if (key=='d') {
//        //        doThemeColorsWindow = !doThemeColorsWindow;
//        //        ofLogVerbose("GreatSand") << "doThemeColorsWindow: " << doThemeColorsWindow ;
//    } else if (key=='f') {
//        //        doSetTheme = !doSetTheme;
//        //        ofLogVerbose("GreatSand") << "doSetTheme: " << doSetTheme ;
//    } else if (key=='u') {
//        basePlaneNormal.rotate(-1, ofVec3f(1,0,0)); // Rotate the base plane normal
//        setRangesAndBasePlaneEquation();
//    } else if (key=='i') {
//        basePlaneNormal.rotate(1, ofVec3f(1,0,0)); // Rotate the base plane normal
//        setRangesAndBasePlaneEquation();
//    } else if (key=='o') {
//        basePlaneNormal.rotate(-1, ofVec3f(0,1,0)); // Rotate the base plane normal
//        setRangesAndBasePlaneEquation();
//    } else if (key=='p') {
//        basePlaneNormal.rotate(1, ofVec3f(0,1,0)); // Rotate the base plane normal
//        setRangesAndBasePlaneEquation();
//    } else if (key=='n') {
//        computeBasePlane();
//    } else if (key=='c') {
//        if (pairsKinect.size() == 0) {
//            ofLogVerbose("GreatSand") << "keyPressed(): Cannot calibrate: No points acquired !!" ;
//        } else {
//            ofLogVerbose("GreatSand") << "keyPressed(): calibrating" ;
//            kpt.calibrate(pairsKinect, pairsProjector);
//            kinectProjMatrix = kpt.getProjectionMatrix();
//            saved = false;
//            loaded = false;
//            calibrated = true;
//        }
//    } else if (key=='r') {
//        generalState = GENERAL_STATE_CALIBRATION;
//        calibrationState = CALIBRATION_STATE_ROI_DETERMINATION;
//        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
//        ofLogVerbose("GreatSand") << "keyPressed(): Finding ROI" ;
//    } else if (key=='m') {
//        generalState = GENERAL_STATE_CALIBRATION;
//        calibrationState = CALIBRATION_STATE_ROI_MANUAL_SETUP;
//        ROICalibrationState = ROI_CALIBRATION_STATE_INIT;
//        ofLogVerbose("GreatSand") << "keyPressed(): Updating ROI Manually" ;
//    } else if (key=='y') {
//        generalState = GENERAL_STATE_CALIBRATION;
//        calibrationState = CALIBRATION_STATE_AUTOCALIB;
//        autoCalibState = AUTOCALIB_STATE_INIT_POINT;
//        ofLogVerbose("GreatSand") << "keyPressed(): Starting autocalib" ;
//    } else if (key=='t') {
//        generalState = GENERAL_STATE_CALIBRATION;
//        calibrationState = CALIBRATION_STATE_CALIBRATION_TEST;
//    } else if (key=='b') {
//        generalState = GENERAL_STATE_SANDBOX;
//    }else if (key=='e') {
//        waitingToInitialiseVehicles = true;
//        generalState = GENERAL_STATE_GAME1;
//    } else if (key=='v') {
//        if (kpt.saveCalibration("calibration.xml"))
//        {
//            ofLogVerbose("GreatSand") << "keyPressed(): Calibration saved " ;
//            saved = true;
//        } else {
//            ofLogVerbose("GreatSand") << "keyPressed(): Calibration could not be saved " ;
//        }
//    } else if (key=='l') {
//        if (kpt.loadCalibration("calibration.xml"))
//        {
//            ofLogVerbose("GreatSand") << "keyPressed(): Calibration loaded " ;
//            kinectProjMatrix = kpt.getProjectionMatrix();
//            loaded = true;
//            calibrated = true;
//        } else {
//            ofLogVerbose("GreatSand") << "keyPressed(): Calibration could not be loaded " ;
//        }
//    } else if (key=='j') {
//        if (saveSettings("settings.xml"))
//        {
//            ofLogVerbose("GreatSand") << "keyPressed(): Settings saved " ;
//        } else {
//            ofLogVerbose("GreatSand") << "keyPressed(): Settings could not be saved " ;
//        }
//    } else if (key=='k') {
//        if (loadSettings("settings.xml"))
//        {
//            ofLogVerbose("GreatSand") << "keyPressed(): Settings loaded " ;
//            setRangesAndBasePlaneEquation();
//            updateKinectGrabberROI();
//        } else {
//            ofLogVerbose("GreatSand") << "keyPressed(): Settings could not be loaded " ;
//        }
//    }
//    
//    if (key=='r'|| key== 'm' || key== 'y'|| key=='t' || key=='b' || key == 'e') {
//        firstImageReady = false;
//        updateMode();
//    }
}

void ofApp::keyReleased(int key){
    
}

void ofApp::mouseMoved(int x, int y ){
    
}

void ofApp::mouseDragged(int x, int y, int button){
    
}

void ofApp::mousePressed(int x, int y, int button){
//    if (generalState == GENERAL_STATE_CALIBRATION && calibrationState == CALIBRATION_STATE_ROI_MANUAL_SETUP){
//        // Manual ROI calibration
//        if (ROICalibrationState == ROI_CALIBRATION_STATE_INIT)
//        {
//            kinectROIManualCalib.setPosition(x, y);
//            ROICalibrationState = ROI_CALIBRATION_STATE_MOVE_UP;
//        } else if (ROICalibrationState == ROI_CALIBRATION_STATE_MOVE_UP) {
//            kinectROIManualCalib.setSize(x-kinectROIManualCalib.x, y-kinectROIManualCalib.y);
//            ROICalibrationState = ROI_CALIBRATION_STATE_DONE;
//            
//            kinectROI = kinectROIManualCalib;
//            kinectROI.standardize();
//            ofLogVerbose("GreatSand") << "mousePressed(): kinectROI : " << kinectROI ;
//            updateKinectGrabberROI();
//            //update the mesh
//            setupMesh();
//        }
//    }
}

void ofApp::mouseReleased(int x, int y, int button){
    
}

void ofApp::mouseEntered(int x, int y){
    
}

void ofApp::mouseExited(int x, int y){
    
}

void ofApp::windowResized(int w, int h){
    
}

void ofApp::gotMessage(ofMessage msg){
    
}

void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

void ofApp::setupGui(){
    // instantiate and position the gui //
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT );
    
    // add some components //
//    gui->addTextInput("message", "# open frameworks #");
    
    // add a folder to group a few components together //
    gui->addSlider("# of fish", 0, 10, 0)->setPrecision(0);
    gui->addSlider("# of rabbits", 0, 10, 0)->setPrecision(0);
    gui->addToggle("Mother fish", showMotherFish);
    gui->addToggle("Mother rabbit", showMotherRabbit);
    gui->addButton("Reset animal locations");
    gui->addButton("Remove all animals");
    gui->addBreak();
    gui->addHeader(":: Game ::");
    gui->addFooter();
    
    // once the gui has been assembled, register callbacks to listen for component specific events //
    gui->onButtonEvent(this, &ofApp::onButtonEvent);
    gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->onSliderEvent(this, &ofApp::onSliderEvent);
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

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Reset animal locations")) {
        setupVehicles();
    } else if (e.target->is("Remove all animals")) {
        fish.clear();
        rabbits.clear();
        fishNum = 0;
        rabbitsNum = 0;
    }
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e){
    if (e.target->is("Mother fish")) {
        if (!showMotherFish)
            addMotherFish();
        showMotherFish = e.checked;
    } else if (e.target->is("Mother rabbit")) {
        if (!showMotherRabbit)
            addMotherRabbit();
        showMotherRabbit = e.checked;
    }
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e){
    if (e.target->is("# of fish")) {
        if (e.value > fishNum)
            while (e.value > fishNum){
                addNewFish();
                fishNum++;
            }
        if (e.value < fishNum)
            while (e.value < fishNum){
                fish.pop_back();
                fishNum--;
            }

    } else if (e.target->is("# of rabbits")) {
        if (e.value > rabbitsNum)
            while (e.value > rabbitsNum){
                addNewRabbit();
                rabbitsNum++;
            }
        if (e.value < rabbitsNum)
            while (e.value < rabbitsNum){
                rabbits.pop_back();
                rabbitsNum--;
            }
    }
}



