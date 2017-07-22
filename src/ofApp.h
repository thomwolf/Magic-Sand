/***********************************************************************
ofApp.h - main openframeworks app
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

#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "KinectProjector/KinectProjector.h"
#include "SandSurfaceRenderer/SandSurfaceRenderer.h"
#include "vehicle.h"
#include "Model.h"

class ofApp : public ofBaseApp {

public:
	void setup();

	void update();

	void draw();
	void drawProjWindow(ofEventArgs& args);
	void drawVehicles();
	
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void setupGui();
	void onButtonEvent(ofxDatGuiButtonEvent e);
	void on2dPadEvent(ofxDatGui2dPadEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);

	std::shared_ptr<ofAppBaseWindow> projWindow;

private:
	std::shared_ptr<KinectProjector> kinectProjector;
	SandSurfaceRenderer* sandSurfaceRenderer;
    Model* model;
	
	// Projector and kinect variables
	ofRectangle kinectROI;
	
	// FBos
	ofFbo fboVehicles;
	ofFbo fboInterface;
    ofVec2f firePos;

	//Model Variables
	bool runstate;
    int windSpeed;
    int windDirection;

	// GUI
	ofxDatGui* gui;

    void drawMainWindow(float x, float y, float width, float height);
    void drawWindArrow();
    void drawPositioningTarget(ofVec2f firePos);
};
