/***********************************************************************
Main.cpp
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)

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


#include "ofMain.h"
#include "ofApp.h"

const std::string MagicSandVersion = "1.5.4.2";

ofGLFWWindowSettings setWindowDimensions(GLFWmonitor* monitor, bool mainWindow)
{
	// Create main screen
	ofGLFWWindowSettings settings;

	const GLFWvidmode * desktopMode = glfwGetVideoMode(monitor);
	settings.width = desktopMode->width;
	settings.height = desktopMode->height;

	int xM; int yM;
	glfwGetMonitorPos(monitor, &xM, &yM);
	settings.setPosition(ofVec2f(xM, yM));

	if (mainWindow)
	{
		settings.width *= 4.0 / 5.0;
		settings.height *= 4.0 / 5.0;
		settings.resizable = true;
		settings.decorated = true;
		settings.title = "Magic-Sand " + MagicSandVersion;
	}
	else
	{
		settings.resizable = false;
		settings.decorated = false;
	}
	return settings;
}

//========================================================================
int main() {

	ofGLFWWindowSettings settings;
	settings.visible = false;
	shared_ptr<ofAppBaseWindow> dummyWindow = ofCreateWindow(settings);

	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	cout << "Number of screens found: " << count << endl;

	if (count == 0) {
		cerr << "No monitors found. Exiting" << endl;
		return -1;
	}

	// Set the debug window on Screen 0
	ofGLFWWindowSettings mainWindowSettings = setWindowDimensions(monitors[0], true);
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(mainWindowSettings);

	mainWindow->setWindowPosition(ofGetScreenWidth() / 2 - mainWindowSettings.width / 2, ofGetScreenHeight() / 2 - mainWindowSettings.height / 2);
	mainWindow->setWindowShape(mainWindowSettings.width, mainWindowSettings.height);

	// Set the kinect window on Screen n - 1
	ofGLFWWindowSettings projectorWindowSettings = setWindowDimensions(monitors[count - 1], false);
	projectorWindowSettings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> projectorWindow = ofCreateWindow(projectorWindowSettings);
	projectorWindow->setVerticalSync(false);

	shared_ptr<ofApp> mainApp(new ofApp);
	ofAddListener(projectorWindow->events().draw, mainApp.get(), &ofApp::drawProjWindow);
	mainApp->projWindow = projectorWindow;

	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();
	getchar();
	return 0;
}
