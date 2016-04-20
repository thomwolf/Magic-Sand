#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include <Geometry/HVector.h>
#include <Geometry/Plane.h>
#include <Geometry/Point.h>
#include <Geometry/Matrix.h>
#include <Geometry/ProjectiveTransformation.h>

#include "ColorMap.h"
#include "FrameFilter.h"
#include "KinectGrabber.h"
#include "vehicle.h"
#include "SurfaceRenderer.h"

using namespace cv;


class ofApp : public ofBaseApp{

	public:
	typedef Geometry::Point<double,3> Point;
	typedef Geometry::Vector<double,3> Vector;
	typedef Geometry::Plane<double,3> Plane;
	typedef Geometry::ProjectiveTransformation<double,3> PTransform;

        void setup();
		void update();
		void draw();
        void drawProjWindow(ofEventArgs& args);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

        int generalState, calibrationState;

private:
    ofShader                    shader;            //Shader
    ofFbo                       fbo;			//Buffer for intermediate drawing
    ColorMap                    heightMap;
    KinectGrabber               kinectgrabber;
    SurfaceRenderer*             surfaceRenderer;
    
    ofMesh mesh;
    int meshwidth;          //Mesh size
    int meshheight;

};
