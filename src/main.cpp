#include "ofMain.h"
#include "ofApp.h"

bool setSecondWindowDimensions(ofGLFWWindowSettings& settings) {
	// Check screens size and location
	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	cout << "Number of screens found: " << count << endl;
	if (count>1) {
		int xM; int yM;
		glfwGetMonitorPos(monitors[1], &xM, &yM); // We take the second monitor
		const GLFWvidmode * desktopMode = glfwGetVideoMode(monitors[1]);

		settings.width = desktopMode->width;
		settings.height = desktopMode->height;
		settings.setPosition(ofVec2f(xM, yM));
		return true;
	} else {
		settings.width = 800; // Default settings if there is only one screen
		settings.height = 600;
		settings.setPosition(ofVec2f(0, 0));
		return false;
	}
}

//========================================================================
int main() {
	ofGLFWWindowSettings settings;
	settings.width = 1200;
	settings.height = 600;
	settings.setPosition(ofVec2f(0, 0));
	settings.resizable = true;
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);

	setSecondWindowDimensions(settings);
	settings.resizable = false;
	settings.decorated = false;
	settings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> secondWindow = ofCreateWindow(settings);
	secondWindow->setVerticalSync(false);

	shared_ptr<ofApp> mainApp(new ofApp);
	ofAddListener(secondWindow->events().draw, mainApp.get(), &ofApp::drawProjWindow);

	mainApp->projWindow = secondWindow;

	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();
}
