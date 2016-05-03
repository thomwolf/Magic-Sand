/*
 * KinectGrabber.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: arturo
 */

#include "KinectGrabber.h"
#include "ofConstants.h"

KinectGrabber::KinectGrabber()
:newFrame(true){
	// start the thread as soon as the
	// class is created, it won't use any CPU
	// until we send a new frame to be analyzed
    //	startThread();
}

KinectGrabber::~KinectGrabber(){
	// when the class is destroyed
	// close both channels and wait for
	// the thread to finish
    //	toAnalyze.close();
    //	analyzed.close();
	waitForThread(true);
}

void KinectGrabber::setup(){
    //	// send the frame to the thread for analyzing
    //	// this makes a copy but we can't avoid it anyway if
    //	// we want to update the grabber while analyzing
    //    // previous frames
    
	// settings and defaults
	storedframes = 0;
    //    storedcoloredframes = 0;
    
    kinect.init();
    kinect.setRegistration(true); // So we have correspondance between RGB and depth images
    kinect.open();
    kinect.setUseTexture(false);
    kinectWidth = kinect.getWidth();
    kinectHeight = kinect.getHeight();
    kinectDepthImage.allocate(kinectWidth, kinectHeight, 1);
//    kinectDepthImage.setUseTexture(false);
    kinectColorImage.allocate(kinectWidth, kinectHeight);
    kinectColorImage.setUseTexture(false);
}

void KinectGrabber::setupFramefilter(int sNumAveragingSlots, int gradFieldresolution, float snearclip, float sfarclip,const ofVec3f basePlaneNormal, double MinElevation,double MaxElevation, ofRectangle ROI) {
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
    framefilter.setup(kinectWidth,kinectHeight,sNumAveragingSlots, gradFieldresolution, snearclip, sfarclip, basePlaneNormal, MinElevation, MaxElevation);
    framefilter.setROI(ROI);
//    framefilter.setValidElevationInterval(basePlaneNormal,elevationMin,elevationMax);
    // framefilter.startThread();
}

void KinectGrabber::setupClip(float snearclip, float sfarclip){
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
}

void KinectGrabber::setMode(General_state sgeneralState, Calibration_state scalibrationState){
    generalState = sgeneralState;
    calibrationState = scalibrationState;
    framefilter.resetBuffers();
}

bool KinectGrabber::isFrameNew(){
	return newFrame;
}

ofVec2f KinectGrabber::getKinectSize(){
	return ofVec2f(kinectWidth, kinectHeight);
}

ofMatrix4x4 KinectGrabber::getWorldMatrix(){
    ofVec3f a = kinect.getWorldCoordinateAt(0, 0, 1); // Little to access kinect internal parameters without having to modify ofxKinect
    ofVec3f b = kinect.getWorldCoordinateAt(1, 1, 1);
    cout << "Computing kinect world matrix" << endl;
    return ofMatrix4x4(b.x-a.x, 0, 0, a.x,
                       0,b.y-a.y, 0, a.y,
                       0, 0, 0, 1,
                       0, 0, 0, 1);
}

void KinectGrabber::setKinectROI(ofRectangle skinectROI){
    framefilter.setROI(skinectROI);
}

void KinectGrabber::threadedFunction(){
    // wait until there's a new frame
    // this blocks the thread, so it doesn't use
    // the CPU at all, until a frame arrives.
    // also receive doesn't allocate or make any copies
	while(isThreadRunning()) {
        
        //Update clipping planes of kinect if needed
        float snearclip = nearclip;
        float sfarclip = farclip;
        if(nearclipchannel.tryReceive(snearclip) || farclipchannel.tryReceive(sfarclip)) {
            while(nearclipchannel.tryReceive(snearclip) || farclipchannel.tryReceive(sfarclip)) {
            } // clear queue
            kinect.setDepthClipping(snearclip, sfarclip);
            framefilter.setDepthRange(snearclip, sfarclip);
            framefilter.resetBuffers();
        }

        newFrame = false;
        if (storedframes == 0)
        {
            // If new image in kinect => send to filter thread
            kinect.update();
            
            if(kinect.isFrameNew()){
                newFrame = true;
                if (generalState == GENERAL_STATE_CALIBRATION) {
                    kinectDepthImage = kinect.getDepthPixels();
                    kinectColorImage.setFromPixels(kinect.getPixels());
                    // If new filtered image => send back to main thread
#if __cplusplus>=201103
                    colored.send(std::move(kinectColorImage.getPixels()));
                    filtered.send(std::move(kinectDepthImage));
#else
                    colored.send(kinectColorImage.getPixels());
                    filtered.send(kinectDepthImage);
#endif
                    lock();
                    storedframes += 1;
                    unlock();
                    
                }
                // if the test mode is activated, the settings are loaded automatically (see gui function)
                if (generalState == GENERAL_STATE_SANDBOX) {
                    kinectDepthImage = kinect.getRawDepthPixels();
                    kinectColorImage.setFromPixels(kinect.getPixels());

                    ofFloatPixels filteredframe;//, kinectProjImage;
                    filteredframe = framefilter.filter(kinectDepthImage);
                    filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
//                    wrldcoord = framefilter.getWrldcoordbuffer();
//                    kinectProjImage = convertProjSpace(filteredframe);
//                    kinectProjImage.setImageType(OF_IMAGE_GRAYSCALE);
                    
                    // If new filtered = => send back to main thread
#if __cplusplus>=201103
                    colored.send(std::move(kinectColorImage.getPixels()));
                    filtered.send(std::move(filteredframe));
                    gradient.send(std::move(framefilter.getGradField()));
#else
                    colored.send(kinectColorImage.getPixels());
                    filtered.send(filteredframe);
                    gradient.send(framefilter.getGradField());
#endif
                    lock();
                    storedframes += 1;
                    unlock();
                }
            }
        }
    }
    kinect.close();
}

