#include "ofApp.h"

using namespace ofxCv;
using namespace cv;
using namespace ofxCSG;

void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    
    // Setup kinectProjector
    kinectProjector = std::make_shared<KinectProjector>(projWindow);
    kinectProjector->setup(true);
    
    // Setup sandSurfaceRenderer
    sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector, projWindow);
    sandSurfaceRenderer->setup(true);
    
    // Retrieve variables
    kinectRes = kinectProjector->getKinectRes();
    kinectROI = kinectProjector->getKinectROI();
    projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
    
    fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
    fboVehicles.begin();
    ofClear(0,0,0,255);
    fboVehicles.end();
    
    setupGui();

    // Vehicles
    showMotherFish = false;
    showMotherRabbit = false;
    motherPlatformSize = 30;
}

void ofApp::addNewFish(){
    ofVec2f location;
    location = findRandomVehicleLocation(kinectROI, true);
    auto f = Fish(kinectProjector, location, kinectROI, motherFish);
    f.setup();
    fish.push_back(f);
}

void ofApp::addNewRabbit(){
    ofVec2f location;
    location = findRandomVehicleLocation(kinectROI, false);
    auto r = Rabbit(kinectProjector, location, kinectROI, motherRabbit);
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
    for (auto & f : fish){
        f.setMotherLocation(motherFish);
    }
    showMotherFish = true;
}

void ofApp::addMotherRabbit(){
    int minborderDist = 40;
    ofRectangle internalBorders = kinectROI;
    internalBorders.scaleFromCenter((kinectROI.width-minborderDist)/kinectROI.width, (kinectROI.height-minborderDist)/kinectROI.height);
    
    // Find a location for the Rabbits mother inside of the water to be sure the rabbits cannot reach her without help
    motherRabbit = findRandomVehicleLocation(internalBorders, true);
    
    // Set the mother Rabbit plateform location over the sea level
    motherRabbit.z = kinectProjector->elevationToKinectDepth(10, motherRabbit.x, motherRabbit.y);
    
    for (auto & r: rabbits){
        r.setMotherLocation(motherRabbit);
    }
    showMotherRabbit = true;
}

ofVec2f ofApp::findRandomVehicleLocation(ofRectangle area, bool liveInWater){
    ofVec2f location;
    bool okwater = false;
    while (!okwater) {
        float x = ofRandom(area.getLeft(),area.getRight());
        float y = ofRandom(area.getTop(),area.getBottom());
        bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater)){
            location = ofVec2f(x, y);
            okwater = true;
        }
    }
    return location;
}

void ofApp::update(){
    kinectProjector->update();
    sandSurfaceRenderer->update();

    if (kinectProjector->isImageStabilized()) {
        for (auto & f : fish){
            f.applyBehaviours(showMotherFish);
            f.update();
        }
        for (auto & r : rabbits){
            r.applyBehaviours(showMotherRabbit);
            r.update();
        }
        drawVehicles();
    }
}

void ofApp::draw(){
    sandSurfaceRenderer->drawMainWindow(400, 20, 400, 300);
    fboVehicles.draw(400, 20, 400, 300);
    kinectProjector->drawMainWindow(500, 340, 320, 240);
}

void ofApp::drawProjWindow(ofEventArgs &args){
    kinectProjector->drawProjectorWindow();
    
    if (!kinectProjector->isCalibrating()){
        sandSurfaceRenderer->drawProjectorWindow();
        fboVehicles.draw(0,0);
    }
}

void ofApp::drawVehicles()
{
    fboVehicles.begin();
    ofClear(255,255,255, 0);
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
    fboVehicles.end();
}

void ofApp::drawMotherFish()
{
    // Mother fish scale
    float sc = 10;
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    float tailangle = 0;
    
    ofPushMatrix();
    ofTranslate(kinectProjector->kinectCoordToProjCoord(motherFish.x+tailSize, motherFish.y));
    
    ofNoFill();
    ofSetColor(255);//ofColor::blueSteel);
    ofDrawCircle(-0.5*sc, 0, motherPlatformSize);

    ofNoFill();
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
    
    ofNoFill();
    ofSetColor(255);//ofColor::yellow);
    ofDrawCircle(-5*sc, 0, motherPlatformSize);

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

}

void ofApp::keyReleased(int key){
    
}

void ofApp::mouseMoved(int x, int y ){
    
}

void ofApp::mouseDragged(int x, int y, int button){
    
}

void ofApp::mousePressed(int x, int y, int button){

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
    gui = new ofxDatGui();
    gui->addSlider("# of fish", 0, 10, fish.size())->setPrecision(0);
    gui->addSlider("# of rabbits", 0, 10, rabbits.size())->setPrecision(0);
    gui->addToggle("Mother fish", showMotherFish);
    gui->addToggle("Mother rabbit", showMotherRabbit);
    gui->addButton("Remove all animals");
    gui->addBreak();
    gui->addHeader(":: Game ::", false);

    gui->onButtonEvent(this, &ofApp::onButtonEvent);
    gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->onSliderEvent(this, &ofApp::onSliderEvent);
    gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    
    gui->setPosition(ofxDatGuiAnchor::BOTTOM_RIGHT); // You have to do it at the end
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Remove all animals")) {
        fish.clear();
        rabbits.clear();
        showMotherFish = false;
        showMotherRabbit = false;
        gui->getSlider("# of fish")->setValue(0);
        gui->getSlider("# of rabbits")->setValue(0);
        gui->getToggle("Mother fish")->setChecked(false);
        gui->getToggle("Mother rabbit")->setChecked(false);
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
        if (e.value > fish.size())
            while (e.value > fish.size()){
                addNewFish();
            }
        if (e.value < fish.size())
            while (e.value < fish.size()){
                fish.pop_back();
            }

    } else if (e.target->is("# of rabbits")) {
        if (e.value > rabbits.size())
            while (e.value > rabbits.size()){
                addNewRabbit();
            }
        if (e.value < rabbits.size())
            while (e.value < rabbits.size()){
                rabbits.pop_back();
            }
    }
}



