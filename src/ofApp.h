#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"
#include "KinectProjector.h"
#include "SandSurfaceRenderer.h"
#include "vehicle.h"
#include "Utils.h"

#include "of1App.h"

class ofApp : public ofBaseApp {

public:
	
	void setup();
	void addNewFish();
	void addNewRabbit();
	void addMotherFish();
	void addMotherRabbit();
	ofVec2f findRandomVehicleLocation(ofRectangle area, bool liveInWater);

	void update();

	void draw();
	void drawProjWindow(ofEventArgs& args);
	void drawVehicles();
	void drawMotherFish();
	void drawMotherRabbit();

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
	void onToggleEvent(ofxDatGuiToggleEvent e);
	void onSliderEvent(ofxDatGuiSliderEvent e);

	std::shared_ptr<ofAppBaseWindow> projWindow;

private:
	std::shared_ptr<KinectProjector> kinectProjector;
	SandSurfaceRenderer* sandSurfaceRenderer;
	
	// Projector and kinect variables
	ofVec2f projRes;
	ofVec2f kinectRes;
	ofRectangle kinectROI;
	
	// FBos
	ofFbo fboVehicles;

	// Fish and Rabbits
	vector<Fish> fish;
	vector<Rabbit> rabbits;
	int fishNum;
	int rabbitsNum;
	
	// Fish and Rabbits mothers
	ofPoint motherFish;
	ofPoint motherRabbit;
	bool showMotherFish;
	bool showMotherRabbit;
	float motherPlatformSize;
	bool waitingToInitialiseVehicles;
	
	// GUI
	ofxDatGui* gui;
};
