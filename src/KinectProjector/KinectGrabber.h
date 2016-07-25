/***********************************************************************
 KinectGrabber - control the kinect and perform frame filtering.
 Frame filtering method adapted from Oliver Kreylos SARndbox FrameFilter.cpp
 https://github.com/KeckCAVES/SARndbox
 ***********************************************************************/

#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"

#include "Utils.h"

class KinectGrabber: public ofThread {
public:
	typedef unsigned short RawDepth; // Data type for raw depth values
	typedef float FilteredDepth; // Data type for filtered depth values

	KinectGrabber();
	~KinectGrabber();
    void start();
    void stop();
    void performInThread(std::function<void(KinectGrabber&)> action);
    bool setup();
	bool openKinect();
	void setupFramefilter(int gradFieldresolution, float newMaxOffset, ofRectangle ROI, bool spatialFilter, bool followBigChange, int numAveragingSlots);
    void initiateBuffers(void); // Reinitialise buffers
    void resetBuffers(void);
    
    ofVec3f getStatBuffer(int x, int y);
    float getAveragingBuffer(int x, int y, int slotNum);
    float getValidBuffer(int x, int y);
    
    void setFollowBigChange(bool newfollowBigChange);
    void setKinectROI(ofRectangle skinectROI);
    void setAveragingSlotsNumber(int snumAveragingSlots);
    void setGradFieldResolution(int sgradFieldresolution);
    
    void decStoredframes(){
        storedframes -= 1;
    }
    
    bool isImageStabilized(){
        return firstImageReady;
    }
    
    bool isFrameNew(){
        return newFrame;
    }
    
    ofVec2f getKinectSize(){
        return ofVec2f(width, height);
    }
    
    float getRawDepthAt(int x, int y){
        return kinectDepthImage.getData()[(int)(y*width+x)];
    }
    
	ofMatrix4x4 getWorldMatrix();
    
    int getNumAveragingSlots(){
        return numAveragingSlots;
    }
    
    void setMaxOffset(float newMaxOffset){
        maxOffset = newMaxOffset;
    }
    
    void setSpatialFiltering(bool newspatialFilter){
        spatialFilter = newspatialFilter;
    }
    
	ofThreadChannel<ofFloatPixels> filtered;
	ofThreadChannel<ofPixels> colored;
	ofThreadChannel<ofVec2f*> gradient;
    
private:
	void threadedFunction() override;
    void filter();
    bool isInsideROI(int x, int y); // test is x, y is inside ROI
    void applySpaceFilter();
    void updateGradientField();
    
	bool newFrame;
    bool bufferInitiated;
    bool firstImageReady;
    int storedframes;
    
    // Thread lambda functions (actions)
	vector<std::function<void(KinectGrabber&)> > actions;
	ofMutex actionsLock;
    
    // Kinect parameters
	bool kinectOpened;
    ofxKinect               kinect;
    unsigned int width, height; // Width and height of kinect frames
    int minX, maxX, ROIwidth; // ROI definition
    int minY, maxY, ROIheight;
    
    // General buffers
    ofxCvColorImage         kinectColorImage;
    ofShortPixels     kinectDepthImage;
    ofFloatPixels filteredframe;
    ofVec2f* gradField;
    
    // Filtering buffers
	float* averagingBuffer; // Buffer to calculate running averages of each pixel's depth value
	float* statBuffer; // Buffer retaining the running means and variances of each pixel's depth value
	float* validBuffer; // Buffer holding the most recent stable depth value for each pixel
    
    // Gradient computation variables
    int gradFieldcols, gradFieldrows;
    int gradFieldresolution;           //Resolution of grid relative to window width and height in pixels
    float maxgradfield, depthrange;
    
    // Frame filter parameters
	int numAveragingSlots; // Number of slots in each pixel's averaging buffer
	int averagingSlotIndex; // Index of averaging slot in which to store the next frame's depth values
	unsigned int minNumSamples; // Minimum number of valid samples needed to consider a pixel stable
	float maxVariance; // Maximum variance to consider a pixel stable
    float initialValue;
    float outsideROIValue;
	float hysteresis; // Amount by which a new filtered value has to differ from the current value to update the display
    bool followBigChange;
    float bigChange; // Amount of change over which the averaging slot is reset to new value
	float instableValue; // Value to assign to instable pixels if retainValids is false
	bool spatialFilter; // Flag whether to apply a spatial filter to time-averaged depth values
    float maxOffset;
    
    int minInitFrame; // Minimal number of frame to consider the kinect initialized
    int currentInitFrame;
    
    // Debug
//    int blockX, blockY;
};
