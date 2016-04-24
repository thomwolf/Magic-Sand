#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinectProjectorToolkit.h"

#include "ColorMap.h"
#include "FrameFilter.h"
#include "KinectGrabber.h"
#include "vehicle.h"
#include "SurfaceRenderer.h"
#include "Utils.h"

using namespace cv;


class ofApp : public ofBaseApp{

	public:

        enum General_state
        {
            GENERAL_STATE_CALIBRATION = 0,
            GENERAL_STATE_SANDBOX = 1
        };
        enum Calibration_state
        {
            CALIBRATION_STATE_ROI_DETERMINATION = 0,
            CALIBRATION_STATE_PROJ_KINECT_CALIBRATION = 1
        };

        void setup();
		void update();
		void draw();
        void drawProjWindow(ofEventArgs& args);
        void drawChessboard(int x, int y, int chessboardSize);
        void drawTestingPoint(ofVec2f projectedPoint);
        void addPointPair();
        

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

        shared_ptr<ofAppBaseWindow> projWindow;

        General_state generalState;
        Calibration_state calibrationState;

    private:
        ColorMap                    heightMap;
        KinectGrabber               kinectgrabber;
        SurfaceRenderer*            surfaceRenderer;
        ofRectangle                 kinectROI;
        ofShader                    shader;            //Shader
        ofFbo                       fbo;			//Buffer for intermediate drawing
        ofFbo                       fboChessboard;
        ofxKinectProjectorToolkit   kpt;
    
        ofxCvColorImage             rgbImage;
        cv::Mat                     cvRgbImage;
        ofxCvFloatImage             FilteredDepthImage;
        ofxCvColorImage             kinectColorImage;
    
        vector<ofVec2f>             currentProjectorPoints;
        vector<cv::Point2f>         cvPoints;
        vector<ofVec3f>             pairsKinect;
        vector<ofVec2f>             pairsProjector;
        
        string                      resultMessage;
        ofColor                     resultMessageColor;
        ofVec2f                     testPoint;
        bool saved;
        bool loaded;

        ofMesh mesh;
        int meshwidth;          //Mesh size
        int meshheight;
        ofVec3f basePlaneNormal; // Base plane coord
        ofVec3f basePlaneNOffset;

        double gradFieldresolution;
        
        float farclip, nearclip;
        float elevationMin, elevationMax;
        int                         chessboardSize;
        int                         chessboardX;
        int                         chessboardY;
        int projResX;
        int projResY;
        int kinectResX;
        int kinectResY;
};
