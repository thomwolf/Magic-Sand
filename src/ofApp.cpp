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
	
	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0,0,0,0);
	fboVehicles.end();
	//Initialize interface parameters without slider movement
	firePos.set(kinectROI.width / 2, kinectROI.height / 2);
	model->setTemp(25);
	model->setWindspeed(5);
	model->setWinddirection(180);
	runstate = false;
	setupGui();
}



void ofApp::update() {
    // Call kinectProjector->update() first during the update function()
	kinectProjector->update();
    
	sandSurfaceRenderer->update();
    
    if (kinectProjector->isROIUpdated())
        kinectROI = kinectProjector->getKinectROI();

	if (kinectProjector->isImageStabilized()) {
        if(runstate){
			model->update();
			drawVehicles();
		}
		if (!model->isRunning()) {
			gui->getButton("Start fire")->setLabel("Start fire");
			runstate = false;
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
    fboVehicles.draw(x, y, width, height);
    kinectProjector->drawMainWindow(x, y, width, height);
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
    model->draw();
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

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Start fire")) {
		// Button functionality depending on State
		if (gui->getButton("Start fire")->getLabel() == "Start fire") {
			runstate = true;
			gui->getButton("Start fire")->setLabel("Pause");
			model->addNewFire(firePos);
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
}

void ofApp::on2dPadEvent(ofxDatGui2dPadEvent e) {
	if (e.target->is("Fire position")) {
		firePos.set(e.x, e.y);
	}
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("Temperature")) {
		model->setTemp(e.value);
	}

	if (e.target->is("Wind speed")) {
		model->setWindspeed(e.value);
	}

	if (e.target->is("Wind direction")) {
		model->setWinddirection(e.value);
	}
}
