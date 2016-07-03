#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    ofGLFWWindowSettings settings;
//	settings.setGLVersion(3,2);
    
    settings.width = 800;
    settings.height = 600;
    settings.setPosition(ofVec2f(0, 0));
    settings.resizable = true;
    shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);
    
    settings.width = 800;//PROJECTOR_RESOLUTION_X;
    settings.height = 600;//PROJECTOR_RESOLUTION_Y;
    settings.setPosition(ofVec2f(ofGetScreenWidth(), 0));
    settings.resizable = false;
    settings.decorated = false;
    settings.shareContextWith = mainWindow;
    shared_ptr<ofAppBaseWindow> secondWindow = ofCreateWindow(settings);
    secondWindow->setVerticalSync(false);
    
    shared_ptr<ofApp> mainApp(new ofApp);
    ofAddListener(secondWindow->events().draw, mainApp.get(), &ofApp::drawProjWindow);
    ofAddListener(secondWindow->events().keyPressed, mainApp.get(), &ofApp::keyPressed);
    mainApp->projWindow = secondWindow;
    
    ofRunApp(mainWindow, mainApp);
    ofRunMainLoop();
}
