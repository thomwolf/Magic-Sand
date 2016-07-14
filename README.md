# Magic Sand
Magic Sand is a software for operating an augmented reality sandbox such as the [Augmented Reality Sandbox](https://arsandbox.ucdavis.edu)
developped by [UC Davis](http://idav.ucdavis.edu/~okreylos/ResDev/SARndbox/).

Magic Sand was inspired by the [SARndbox](https://github.com/KeckCAVES/SARndbox) project and
was developed with the specific aim of enabling the use of an augmented reality sandbox in a home environment.

The following considerations thus guided its development:
- operate on a standard home computer (no specific GPU requirement, run on Windows, Mac OS X and Linux).
- automatic calibration, configuration and operation of the Sandbox so that the sandbox can be stored and deployed as quickly as possible.
- user-friendly interface for controling and configuring the sandbox that the sandbox can be operated by the family and friends.
- being a extensible framework for sandbox games and applications development.

##Main Features

Magic Sand operates on a computer connected to a home cinema projector and a kinect sensor.
The software controls the projector to project colors as a function of the sand level measured by the kinect sensor.

Magic Sand transforms the sandbox in a colorful playground.

A simple game is included in which animals (fish and rabbits) populate the sandbox.
The user can help the animals to reach their mothers by digging rivers or building mountains in the sand.

##Special features
Magic Sand was designed to be as user-friendly as possible. It features:
- Automatic calibration of the sandbox.
- User friendly interface for controling the sandbox appearance and the simple game provided.
- Possibility to select, modify, save and import the colormaps.

##Differences with SARndbox
Magic Sand uses the build-in registration feature of the kinect for automatic calibration and does not provide pixel based depth calibration.

It is thus probably less acurate than SARndbox.

Magic Sand does not provide dynamic rain features of SARndbox which require a strong GPU usually not provided on a laptop.

##Operation
The operation of the software is safe-explanatory.

The autocalibration process starts automatically if no calibration file is detected.
The autocalibration process comprises the following steps:

1. The user is asked to flatten the sand surface. The flatten sand surface will be taken as the future sea level of the sandbox.
2. The sand region is detected by looking for the walls of the sandbox.
3. A series of low-level calibration points are acquired on the sand.
4. The user is asked to put a board over the sandbox.
5. A series of high-level calibration points are acquired on the board. A ceiling of the sand box is also defined a few inches over the board.
6. The calibration process is over.

##Source Code
Magic Sand is based on the [openframeworks](https://openframeworks.cc/) framework.

The source code was designed to be easily extendable so that additional games can be easily developed on its basis.
All the calibration and geometrical computations are contained in a `KinectProjector` class wich can be created as a `shared_ptr` to which you give a pointer to the projector windows on construction
```
std::shared_ptr<ofAppBaseWindow> projWindow;
std::shared_ptr<KinectProjector> kinectProjector;

kinectProjector = std::make_shared<KinectProjector>(projWindow);
kinectProjector->setup(true);
```
Here, `setup(true)` indicates that the GUI of the `kinectProjector` should be displayed.
The `KinectProjector` class take care of the communication with the kinect, the filtering and processing of the depth frames and the calibration of the kinect/projector system. It contains all the function to get the depth of the sand at a given location, the slope of the sand at a given location, to convert coordinates between kinect, world and projector space or to bind a texture containing the depth frame to an openGL shader for instance.

The kinectProjector pointer can be shared among the various classes of the software, for instance shared with a SandSurfaceRenderer element (which compute the color image to be projected on the sand):
```
SandSurfaceRenderer* sandSurfaceRenderer;

sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector, projWindow);
sandSurfaceRenderer->setup(true);
```
Here again, `setup(true)` indicates that the GUI of the `sandSurfaceRenderer` should be displayed.
The kinectProjector need to be updated in the update() function and drawn in the projector draw() function:
```
void ofApp::update(){
  kinectProjector->update();
  sandSurfaceRenderer->update();
}
void ofApp::drawProjWindow(ofEventArgs &args){
  kinectProjector->drawProjectorWindow();
    
  if (!kinectProjector->isCalibrating()){
      sandSurfaceRenderer->drawProjectorWindow();
      fboVehicles.draw(0,0);
  }
}
```
Since the kinectProjector need to have control on the projector for the calibration process, you should be careful not to draw things on projector while a calibration is perfomed (hence the `if (!kinectProjector->isCalibrating())`).

Magic Sand itself forms a simple example on how to use the main `KinectProjector` class to make a simple game.
