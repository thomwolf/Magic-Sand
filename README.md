**For Windows/Mac OSX compiled versions and use instructions, please go to the [release page](https://github.com/thomwolf/Magic-Sand/releases/tag/1.1)**

# Magic Sand
Magic Sand is a software for operating an augmented reality sandbox like the [Augmented Reality Sandbox](https://arsandbox.ucdavis.edu)
developped by [UC Davis](http://idav.ucdavis.edu/~okreylos/ResDev/SARndbox/).

It is a partial port of the [SARndbox](https://github.com/KeckCAVES/SARndbox) project of Oliver Kreylos under [openframeworks](openframeworks.cc/) and is also adapted from the [ofxKinectProjectorToolkit](https://github.com/genekogan/ofxKinectProjectorToolkit) by Gene Kogan.

It was developed with the specific aim of simplifying the use of an augmented reality sandbox in a home/family environment :
- run on a mid-range laptop / home computer (Windows / Mac OS X / Linux, minimal GPU requirement).
- self-calibration to easily disassemble and reassemble the sandbox.
- simple, easy-to-use interface.
- framework for future sandbox-based games and applications.

##Main Features

Operates on a computer connected to a home cinema projector and a kinect sensor.
The software controls the projector to project colors as a function of the sand level measured by the kinect sensor and transforms a sandbox in a colorful playground.

A simple game is included in which animals (fish and rabbits) populate the sandbox.
The user can help the animals to reach their mothers by digging rivers or building mountains in the sand.

##Main differences with [SARndbox](https://github.com/KeckCAVES/SARndbox)
Magic Sand uses the build-in registration feature of the kinect to perform an automatic calibration between the projector and the kinect sensor and does not use a pixel based depth calibration.

It is thus probably less acurate than SARndbox.

Magic Sand does not provide dynamic rain features (typically require a stronger GPU than the graphic card provided on a laptop).

##Source Code
###Dependencies
Magic Sand is based on [openframeworks](openframeworks.cc/) release 0.9.3 and makes use of the following addons:
- official addons (included in openframeworks 0.9.3)
  * ofxOpenCv
  * ofxKinect
  * ofxXmlSettings
- community addons:
  * [ofxCv](https://github.com/kylemcdonald/ofxCv)
  * [ofxParagraph](https://github.com/braitsch/ofxParagraph)
  * [ofxDatGui (forked version)](https://github.com/thomwolf/ofxDatGui)
  * [ofxModal](https://github.com/braitsch/ofxModal)

###Quick start for editing the source code
- Grab a copy of [openframeworks](http://openframeworks.cc/download/) for your OS.
- Grab the additionnal community addons listed above.
- If you are a windows user, install the kinect drivers as detailed on the [release page](https://github.com/thomwolf/Magic-Sand/releases/tag/1.1)
- Enjoy ! (Xcode / VS2015 project files are supplied, should work also on linux and probably Raspberry-Pi but I haven't had time to try it yet)

Be sure to check the [openframeworks](http://openframeworks.cc/) documentation and forum if you don't know it yet, it is an amazing community !

### How it can be used
The code was designed trying to be easily extendable so that additional games/apps can be developed on its basis.

The `KinectProjector` class handles the communication with the kinect sensor, the calibration and the coordinates conversions between kinect (2D), world (3D) and projector (2D) coordinate systems.

You can create a `KinectProjector` object as a `shared_ptr` in the `setup()` function of your openframeworks app. It requires a pointer to the projector window (see provided `main.cpp` on how to properly setup two windows in openframeworks and get a pointer to the projector window).

The `kinectProjector` object can be shared among the various objects that need access to depth and conversions functions (not multi-thread proof of course).

For instance, a `SandSurfaceRenderer` object can be constructed with a pointer to the `kinectProjector` shared object. (the `SandSurfaceRenderer` class convert depth information in color using a editable colormap and display these colors on the sand).

A typical `setup()` function of a openframeworks app can thus reads:
```
std::shared_ptr<ofAppBaseWindow> projWindow;
std::shared_ptr<KinectProjector> kinectProjector;
SandSurfaceRenderer* sandSurfaceRenderer;

void ofApp::setup() {
	kinectProjector = std::make_shared<KinectProjector>(projWindow);
	kinectProjector->setup(true);

	sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector, projWindow);
	sandSurfaceRenderer->setup(true);
}
```
`setup(true)` indicates that the GUI of the `kinectProjector` and the `sandSurfaceRenderer` will be displayed.

The `kinectProjector` object then needs to be updated in the `update()` function of the openframeworks app (preferably before the objects that use its functions) and drawn in the projector `draw()` function.

The `kinectProjector` object needs full control on the projector window during the calibration process so you should be careful not to draw things on the projector window after the call to `kinectProjector->drawProjectorWindow()` if a calibration is currently performed (you can check `kinectProjector->isCalibrating()`).

The following example illustrates the `update()` and `draw()` functions to implement a simple augmented reality sandbox once the `kinectProjector` and `sandSurfaceRenderer` objects have been initiated as detailed above and provided that the projector window has a listener callback setup to the `drawProjWindow(ofEventArgs &args)` function (see `main.cpp`).

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

The source code of Magic Sand itself is a simple example on how to use the main `KinectProjector` class to make a simple game.

###kinectProjector Functions

####Shader functions
The `sandSurfaceRenderer` class shows example of shaders that can be used to compute color and how to set uniforms.

The following function of `KinectProjector` are of special interest to setup a uniform.

```
void bind();
void unbind();
ofMatrix4x4 getTransposedKinectWorldMatrix();
ofMatrix4x4 getTransposedKinectProjMatrix();
```
The `sampler2DRect` received in the shader is normalized between 0 and 1, a conversion scale thus has to be also sent.

####Coordinate conversion / elevation functions
Three coordinate systems can be used:
- the kinect coordinate system of the 2D kinect image : (x, y) in pixel units with origin in the top-left corner,
- the world coordinate system: a 3D coordinate system (x, y, z) in millimeters units originating from the kinect sensor with z axis extending from the kinect sensor, x the horizontal axis of the kinect sensor and y the vertical axis, and
- the projector coordinate system of the 2D projector image : (x, y) in pixel units with origin in the top-left corner.

The most straighforward conversion goes from kinect coordinates to world coordinate system and projector coordinate system.
If you want to animate or display objects, a natural choice would thus be to store then in kinect coordinate and to perform the conversion on display.

The following functions provide conversions between the coordinate systems:
```
ofVec2f worldCoordToProjCoord(ofVec3f vin);
ofVec3f projCoordAndWorldZToWorldCoord(float projX, float projY, float worldZ);
ofVec2f kinectCoordToProjCoord(float x, float y);
ofVec3f kinectCoordToWorldCoord(float x, float y);
ofVec2f worldCoordTokinectCoord(ofVec3f wc);
```

Another value that can be used is the `elevation` which is the distance from a point in world coordinate to a 3D base plane of that is defined by:
- a normal (`getBasePlaneNormal()`) and an offset (`getBasePlaneOffset()`), or
- a plane equation (`getBasePlaneEq()`).

`elevation` can be converted/accessed by the following functions:
```
float elevationAtKinectCoord(float x, float y);
float elevationToKinectDepth(float elevation, float x, float y);
```

`KinectProjector` also store a matrix of gradients of the kinect depth in the world coordinate system (slope of the sand) computed with a given resolution (with a 10 pixels bin by default).
The gradient at a given location can be accessed by:
```
ofVec2f gradientAtKinectCoord(float x, float y);
```

####Setup & calibration functions
`startFullCalibration()` perfoms an automatic calibration of the kinect and the projector.
An automatic calibration comprises:
- ask the user to flatten the sand,
- detecte the sand region by identifying vertical walls of the sandbox,
- measure the average plane formed by the sand surface to define the base plane (see above),
- display and find 5 chess boards (60 calibration points) on the sand surface,
- ask the user to cover the sand with a board,
- display and find 5 chess boards (60 calibration points) on the board surface,
- set the detection ceiling to 50 milimeters above the board.

The following functions can be called to change some internal values of `kinectProjector`:
- `setGradFieldResolution(int gradFieldResolution)`: change the resolution of the gradient field
- `setSpatialFiltering(bool sspatialFiltering)`: toggle the spatial filtering of the depth frame
- `setFollowBigChanges(bool sfollowBigChanges)`: toggle "big change" detection (follow the hand of the user).

####Kinect projector state functions

The following functions give information of the state of the kinectprojector object:
- `isCalibrating()`: is the `kinectProjector` currently performing a calibration 
- `isCalibrated()`: is the `kinectProjector` calibrated (calibration file found or calibration performed)
- `isImageStabilized()`: is the depth frame stabilized (arbitrary time frame after initialisation)
- `isBasePlaneUpdated()`: was the base plane updated in the previous call to `update()'
- `isROIUpdated()`: was the sand region location/extension updated in the previous call to `update()' 
- `isCalibrationUpdated()`: was the calibration updated in the previous call to `update()'

####Kinect projector other getters
The following functions give additional information :
- `getKinectROI()`: get the sand region location/extension 
- `getKinectRes()`: get the kinect resolution 
- `getBasePlaneNormal()` : see above
- `getBasePlaneOffset()` : see above
- `getBasePlaneEq()` : see above
