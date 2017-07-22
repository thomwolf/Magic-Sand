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
	ofSetFrameRate(15);
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

    // Setup Model
    model = new Model(kinectProjector);

	// Retrieve variables
	ofVec2f projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
    kinectROI = kinectProjector->getKinectROI();

	//Interface FBO 
	fboInterface.allocate(projRes.x, projRes.y, GL_RGBA);
	fboInterface.begin();
	ofClear(0, 0, 0, 0);
	fboInterface.end();

	//Vehicles FBO
	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0,0,0,0);
	fboVehicles.end();

	//Initialize interface parameters without slider movement
    runstate = false;
	firePos.set(kinectROI.width / 2, kinectROI.height / 2);
    windSpeed = 5;
    windDirection = 180;

	model->setWindSpeed(windSpeed);
	model->setWindDirection(windDirection);

	setupGui();
}

void ofApp::update() {
    // Call kinectProjector->update() first during the update function()
	kinectProjector->update();
    
	sandSurfaceRenderer->update();
    
    if (kinectProjector->isROIUpdated())
        kinectROI = kinectProjector->getKinectROI();

    if (!model->isRunning()) {
        gui->getButton("Start fire")->setLabel("Start fire");
        runstate = false;
    }

	if (kinectProjector->isImageStabilized()) {
		drawWindArrow();

        if(runstate){
			model->update();
			drawVehicles();
		}
	}
	gui->update();
}

void ofApp::draw() {
    drawMainWindow(300, 30, 600, 450);
	gui->draw();
}

void ofApp::drawMainWindow(float x, float y, float width, float height){
    sandSurfaceRenderer->drawMainWindow(x, y, width, height);
    kinectProjector->drawMainWindow(x, y, width, height);
	fboVehicles.draw(x, y, width, height);
	fboInterface.draw(x, y, width, height);
}

void ofApp::drawProjWindow(ofEventArgs &args) {
	kinectProjector->drawProjectorWindow();
	
	if (!kinectProjector->isCalibrating()){
	    sandSurfaceRenderer->drawProjectorWindow();
	    fboVehicles.draw(0,0);
		fboInterface.draw(0, 0);
	}
}

void ofApp::drawVehicles()
{
    fboVehicles.begin();
    model->draw();
    fboVehicles.end();
}

void ofApp::drawWindArrow()
{
	fboInterface.begin();
    
	ofClear(0, 0, 0, 0);
    
	ofVec2f projectorCoord = kinectProjector->kinectCoordToProjCoord(75, 125);
	ofTranslate(projectorCoord);
	ofRotate(windDirection);
	ofScale(windSpeed/5, 1, 1);
	ofFill();
    
	ofDrawArrow(ofVec3f(20, 2.5, 0), ofVec3f(50, 2.5, 0), 15.05);
	ofPath arrow;
	arrow.rectangle(ofPoint(0, 0), 50, 5);
    arrow.setFillColor(ofColor::white);
	arrow.setStrokeWidth(0);
	arrow.draw();

	ofNoFill();
	fboInterface.end();
}

void ofApp::drawPositioningTarget(ofVec2f firePos) 
{
	fboVehicles.begin();
	ofClear(0, 0, 0, 0);
	ofVec2f projectorCoord = kinectProjector->kinectCoordToProjCoord(firePos.x, firePos.y);

	ofFill();

	ofPath flame;
	flame.circle(projectorCoord.x, projectorCoord.y, 10);
    flame.setFillColor(ofColor::red);
	flame.setStrokeWidth(0);
	flame.draw();

	ofNoFill();
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
	gui->addButton("Calculate Risk Zones");
	gui->add2dPad("Fire position", kinectROI);
	gui->addSlider("Wind speed", 0, 10, windSpeed);
	gui->addSlider("Wind direction", 0, 360, windDirection);
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

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Start fire")) {
		// Button functionality depending on State
		if (gui->getButton("Start fire")->getLabel() == "Start fire") {
			runstate = true;
			// Clear vehicles FBO of target arrow
			fboVehicles.begin();
			ofClear(0, 0, 0, 0);
			fboVehicles.end();
			// Start fire
			model->addNewFire(firePos);
			gui->getButton("Start fire")->setLabel("Pause");
		}
		else if (gui->getButton("Start fire")->getLabel() == "Pause") {
			runstate = false;
			gui->getButton("Start fire")->setLabel("Resume");
		}
		else if (gui->getButton("Start fire")->getLabel() == "Resume") {
			runstate = true;
			gui->getButton("Start fire")->setLabel("Pause");
		}
	}

	if (e.target->is("Reset")) {
		model->clear();
		fboVehicles.begin();
		ofClear(0, 0, 0, 0);
		fboVehicles.end();
		gui->getButton("Start fire")->setLabel("Start fire");
		gui->get2dPad("Fire position")->reset();
		runstate = false;
		
	}

	if (e.target->is("Calculate Risk Zones")) {
		model->calculateRiskZones();
		fboVehicles.begin();
		ofClear(0, 0, 0, 0);
		model->drawRiskZones();
		fboVehicles.end();
	}
}

void ofApp::on2dPadEvent(ofxDatGui2dPadEvent e) {
	if (e.target->is("Fire position")) {
		firePos.set(e.x, e.y);
		if (!model->isRunning()) {
			drawPositioningTarget(firePos);
		}
	}
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("Wind speed")) {
		model->setWindSpeed(e.value);
		windSpeed = e.value;
	}

	if (e.target->is("Wind direction")) {
		model->setWindDirection(e.value);
		windDirection = e.value;		
	}
}
