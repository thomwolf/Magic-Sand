/*
 * KinectGrabber.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: arturo
 */

#include "KinectGrabber.h"
#include "ofConstants.h"
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/GeometryValueCoders.h>

#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480

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
	enableCalibration = false;
	enableTestmode	  = true;
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

void KinectGrabber::setupFramefilter(int sNumAveragingSlots, int gradFieldresolution, float snearclip, float sfarclip,const FrameFilter::PTransform& depthProjection,const FrameFilter::Plane& basePlane, double elevationMin, double elevationMax) {
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
    framefilter.setup(kinectWidth,kinectHeight,sNumAveragingSlots, gradFieldresolution, snearclip, sfarclip, depthProjection, basePlane);
    framefilter.setValidElevationInterval(depthProjection,basePlane,elevationMin,elevationMax);
    // framefilter.startThread();
}

void KinectGrabber::setupClip(float snearclip, float sfarclip){
    //	// send the frame to the thread for analyzing
    //	// this makes a copy but we can't avoid it anyway if
    //	// we want to update the grabber while analyzing
    //    // previous frames
    
    //    if (framefilter.isThreadRunning()){
    //    }
    nearclip =snearclip;
    farclip =sfarclip;
    kinect.setDepthClipping(snearclip, sfarclip);
}

void KinectGrabber::setTestmode(){
    enableTestmode = true;
    enableCalibration = false;
    framefilter.resetBuffers();
}

void KinectGrabber::setCalibrationmode(){
    enableCalibration = true;
    enableTestmode = false;
    //    kinectProjectorOutput.load("kinectProjector.yml");
}

bool KinectGrabber::isFrameNew(){
	return newFrame;
}

void KinectGrabber::setKinectROI(ofRectangle skinectROI){
    kinectROI = skinectROI;
}

KinectGrabber::PTransform KinectGrabber::getProjMatrix(void) {
    double shift_scale = 10; // To shift from cm (native Kinect data) to mm
	float referencePixelSize = kinect.getZeroPlanePixelSize();
    float referenceDistance = kinect.getZeroPlaneDistance();
    float dcmosEmitterDist = kinect.getSensorEmitterDistance();
    float constantShift = kinect.getConstantShift();
    
//	ofMatrix4x4 depthMatrix = ofMatrix4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    /* Calculate the depth-to-distance conversion formula: */
    double numerator=(4.0*shift_scale*double(dcmosEmitterDist)*double(referenceDistance))/double(referencePixelSize);
    double denominator=4.0*double(dcmosEmitterDist)/double(referencePixelSize)+4.0*double(constantShift)+1.5;
    
    /* Calculate the unprojection scale factor: */
    double scale=2.0*double(referencePixelSize)/double(referenceDistance);
    
    /* Construct the depth pixel unprojection matrix: */
    PTransform depthProjection;

    PTransform::Matrix& depthMatrix=depthProjection.getMatrix();
    
    //depthMatrix=PTransform::Matrix::zero;
    // With depth compensation and registration
//    depthMatrix(0,0)=scale;
//    depthMatrix(0,1)=0;
//    depthMatrix(0,2)=0;
//    depthMatrix(0,3)=-scale*double(DEPTH_X_RES)*0.5;
//    depthMatrix(1,0)=0;
//    depthMatrix(1,1)=scale;
//    depthMatrix(1,2)=0;
//    depthMatrix(1,3)=-scale*double(DEPTH_Y_RES)*0.5;
//    depthMatrix(2,0)=0;
//    depthMatrix(2,1)=0;
//    depthMatrix(2,2)=0;
//    depthMatrix(2,3)=-1.0;
//    depthMatrix(3,0)=0;
//    depthMatrix(3,1)=0;
//    depthMatrix(3,2)=-1.0/numerator;
//    depthMatrix(3,3)=denominator/numerator;
    
    // With depth already compensated and no registration
    depthMatrix(0,0)=scale;
    depthMatrix(0,1)=0;
    depthMatrix(0,2)=0;
    depthMatrix(0,3)=-scale*double(DEPTH_X_RES)*0.5;
    depthMatrix(1,0)=0;
    depthMatrix(1,1)=scale;
    depthMatrix(1,2)=0;
    depthMatrix(1,3)=-scale*double(DEPTH_Y_RES)*0.5;
    depthMatrix(2,0)=0;
    depthMatrix(2,1)=0;
    depthMatrix(2,2)=0;
    depthMatrix(2,3)=-1.0;
    depthMatrix(3,0)=0;
    depthMatrix(3,1)=0;
    depthMatrix(3,2)=0;
    depthMatrix(3,3)=1;
    return depthProjection;
}

ofVec3f KinectGrabber::getProjVector(void) {
 	float referencePixelSize = kinect.getZeroPlanePixelSize();
    float referenceDistance = kinect.getZeroPlaneDistance();

    /* Calculate the unprojection scale factor: */
    float scale=2.0*referencePixelSize/referenceDistance;
    
    // With depth already compensated and no registration
    return ofVec3f(float(DEPTH_X_RES)*0.5, float(DEPTH_Y_RES)*0.5, scale);
}

//ofPixels KinectGrabber::convertProjSpace(ofPixels inputframe){
//    // Create a new output frame: */
//    ofPixels newOutputFrame;
//    newOutputFrame.allocate(projWidth, projHeight, 1);
//    newOutputFrame.set(0);
//    
////    unsigned char* ifPtr=static_cast<unsigned char*>(inputframe.getData());
////    unsigned char* nofPtr=static_cast<unsigned char*>(newOutputFrame.getData());
////    
////    ofPoint v1, v2; // v1.x is 0, v1.y is 0, v1.z is 0
////    float z;
////    int ind, val;
////    
////    for(unsigned int y=0;y<kinectHeight;y = y + 1)
////    {
////        for(unsigned int x=0;x<kinectWidth;x = x + 1)
////        {
////            //float z  = farclip;//+nearclip)/2;
////            //cout << "iptr: " << (int)ifPtr[y*kinectWidth+x] << endl;
////            val = ifPtr[y*kinectWidth+x];
////            if (val != 0 && val != 255) {
////                z = (255.0-(float)val)/255.0*(farclip-nearclip)+nearclip;
////                v1.set(x, y, z);// = ofPoint(
////                v2 = kinectProjectorOutput.projectFromDepthXYZ(v1);
//////                cout << "v1: " << v1 << endl;
//////                cout << "v2: " << v2 << endl;
////                if (v2.y >= 0 && v2.y < 600 && v2.x >=0 && v2.x < 800) {
////                    ind = (int)floorf(v2.y)*800+(int)floorf(v2.x);
////                    nofPtr[ind]=val;
////                }
////            }
////        }
////    }
//        newOutputFrame = inputframe;
//    return newOutputFrame;
//}

//ofSetColor(255, 190, 70);
//ofPoint cent = ofPoint(projectorWidth/2, projectorHeight/2);
//for (int i = 0; i < contourFinder.size(); i++) {
//
//    ofPolyline blobContour = contourFinder.getPolyline(i);
//    if(!blobContour.isClosed()){
//        blobContour.close();
//    }
//
//    //if (!blobContour.inside(cent)) {
//    ofPolyline rect = blobContour.getResampledByCount(8);
//    ofBeginShape();
//    kinectgrabber.lock();
//    for (int j = 0; j < rect.size() - 1; j++) {
//        rect[j].z = (farclip-nearclip)*highThresh+nearclip;
//        ofPoint wrld = kinectgrabber.kinectWrapper->getWorldFromRgbCalibratedXYZ(rect[j], true,true);
//        ofPoint currVertex = kinectgrabber.kinectProjectorOutput.projectFromDepthXYZ(rect[j]);
//        ofVertex(currVertex.x, currVertex.y);
//        //					cout << "blob j: "<< j << " rect: "<< rect[j] << " wrld: " << wrld << " currVertex : " << currVertex << endl;
//    }
//    kinectgrabber.unlock();
//    ofEndShape();
//    //}

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
                if (enableCalibration) {
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
                if (enableTestmode) {
                    kinectDepthImage = kinect.getRawDepthPixels();
                    ofFloatPixels filteredframe;//, kinectProjImage;
                    filteredframe = framefilter.filter(kinectDepthImage, kinectROI);
                    filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
//                    wrldcoord = framefilter.getWrldcoordbuffer();
//                    kinectProjImage = convertProjSpace(filteredframe);
//                    kinectProjImage.setImageType(OF_IMAGE_GRAYSCALE);
                    
                    // If new filtered image => send back to main thread
#if __cplusplus>=201103
                    filtered.send(std::move(filteredframe));
                    gradient.send(std::move(framefilter.getGradField()));
#else
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

