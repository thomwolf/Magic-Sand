/***********************************************************************
ofApp.cpp - main openframeworks app
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
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "ofApp.h"

void ofApp::setup() {
	// OF basics
	ofSetFrameRate(60);
	ofBackground(0);
	ofSetVerticalSync(true);
//	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetLogLevel("ofThread", OF_LOG_WARNING);

	// Setup kinectProjector
	kinectProjector = std::make_shared<KinectProjector>(projWindow);
	kinectProjector->setup(true);
	
	// Setup sandSurfaceRenderer
	sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector, projWindow);
	sandSurfaceRenderer->setup(true);
	
	// Retrieve variables
	kinectRes = kinectProjector->getKinectRes();
	projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
	kinectROI = kinectProjector->getKinectROI();
	
	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0,0,0,255);
	fboVehicles.end();
	
	setupGui();
}

void ofApp::addNewFire(){
    ofVec2f location;
	ofVec2f fireSpawnPos;
	fireSpawnPos.set(60, 60);
    setRandomVehicleLocation(kinectROI, false, location);
    auto r = Fire(kinectProjector, location, kinectROI);
    r.setup();
    fires.push_back(r);
}

void ofApp::addNewFire(ofVec2f fireSpawnPos) {
	ofVec2f location;
	setFixedVehicleLocation(fireSpawnPos, false, location);
	auto r = Fire(kinectProjector, location, kinectROI);
	r.setup();
	fires.push_back(r);
}

//Fixed Position for Rabbits : Simon
bool ofApp::setFixedVehicleLocation(ofVec2f pos, bool liveInWater, ofVec2f & location){
	bool okwater = false;
	int countFixed = 0;
	int maxCount = 100;
	while (!okwater && countFixed < maxCount) {
		countFixed++;
		bool insideWater = kinectProjector->elevationAtKinectCoord(pos.x, pos.y) < 0;
		if ((insideWater && liveInWater) || (!insideWater && !liveInWater)) {
			location = pos;
			okwater = true;
		}
	}
	return okwater;
	}


bool ofApp::setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location){
    bool okwater = false;
    int count = 0;
    int maxCount = 100;
    while (!okwater && count < maxCount) {
        count++;
        float x = ofRandom(area.getLeft(),area.getRight());
        float y = ofRandom(area.getTop(),area.getBottom());
        bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater)){
            location = ofVec2f(x, y);
            okwater = true;
        }
    }
    return okwater;
}

void ofApp::update() {
    // Call kinectProjector->update() first during the update function()
	kinectProjector->update();
    
	sandSurfaceRenderer->update();
    
    if (kinectProjector->isROIUpdated())
        kinectROI = kinectProjector->getKinectROI();

	if (kinectProjector->isImageStabilized()) {
	    for (auto & r : fires){
	        r.applyBehaviours();
	        r.update();
	    }
	    drawVehicles();
	}
	gui->update();
}


void ofApp::draw() {
	sandSurfaceRenderer->drawMainWindow(300, 30, 600, 450);//400, 20, 400, 300);
	fboVehicles.draw(300, 30, 600, 450);
	kinectProjector->drawMainWindow(300, 30, 600, 450);
	gui->draw();
}

void ofApp::drawProjWindow(ofEventArgs &args) {
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
    for (auto & r : fires){
        r.draw();
    }
    fboVehicles.end();
}

void ofApp::keyPressed(int key) {

}

void ofApp::keyReleased(int key) {

}

void ofApp::mouseMoved(int x, int y) {

}

void ofApp::mouseDragged(int x, int y, int button) {

}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::mouseReleased(int x, int y, int button) {

}

void ofApp::mouseEntered(int x, int y) {

}

void ofApp::mouseExited(int x, int y) {

}

void ofApp::windowResized(int w, int h) {

}

void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::setupGui(){
	
	//Fire Simulation GUI : Simon
	gui = new ofxDatGui();
	gui->add2dPad("Fire position", kinectROI);
	gui->addSlider("Temperature", 0, 50);
	gui->addSlider("Moisture of soil", 0, 100);
	gui->addSlider("Wind speed", 0, 10);
	gui->addSlider("Wind direction", 0, 360);
	gui->addButton("Start fire");
	gui->addButton("Reset");
	gui->addHeader(":: Fire simulation ::", false);

	// once the gui has been assembled, register callbacks to listen for component specific events //
	gui->onButtonEvent(this, &ofApp::onButtonEvent);
	gui->on2dPadEvent(this, &ofApp::on2dPadEvent);
	gui->onSliderEvent(this, &ofApp::onSliderEvent);
    
    gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    gui->setPosition(ofxDatGuiAnchor::BOTTOM_RIGHT);
	
	gui->setAutoDraw(false); // troubles with multiple windows drawings on Windows
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e){
	if (e.target->is("Start fire")) {
		addNewFire(firePos);
	}
	if (e.target->is("Reset")) {
		fires.clear();
		gui->get2dPad("Fire position")->reset();
	}
}

void ofApp::on2dPadEvent(ofxDatGui2dPadEvent e) {
	if (e.target->is("Fire position")) {
		firePos.set(e.x, e.y);
	}
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e){
}
