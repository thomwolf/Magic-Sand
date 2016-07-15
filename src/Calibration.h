//
//  Calibration.h
//  Magic Sand
//
//  Created by Thomas Wolf on 15/07/16.
//
//

#ifndef __Magic_Sand__Calibration__
#define __Magic_Sand__Calibration__

#include <iostream>
#include "ofMain.h"
#include "matrix.h"
#include "matrix_qr.h"


class ofxKinectProjectorToolkit
{
public:
    ofxKinectProjectorToolkit();
    
    void calibrate(vector<ofVec3f> pairsKinect,
                   vector<ofVec2f> pairsProjector);
    
    ofVec2f getProjectedPoint(ofVec3f worldPoint);
    ofMatrix4x4 getProjectionMatrix();
    vector<ofVec2f> getProjectedContour(vector<ofVec3f> *worldPoints);
    
    vector<double> getCalibration();
    
    bool loadCalibration(string path);
    bool saveCalibration(string path);
    
    bool isCalibrated() {return calibrated;}
    
private:
    
    dlib::matrix<double, 0, 11> A;
    dlib::matrix<double, 0, 1> y;
    dlib::matrix<double, 11, 1> x;
    
    ofMatrix4x4 projMatrice;
    
    bool calibrated;
};

#endif /* defined(__Magic_Sand__Calibration__) */

