/***********************************************************************
KinectProjectorCalibration.cpp - KinectProjectorCalibration compute
the calibration of the kinect and projector.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxKinectProjectorToolkit by Gene Kogan:
https://github.com/genekogan/ofxKinectProjectorToolkit
Copyright (c) 2014 Gene Kogan

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
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef __Magic_Sand__Calibration__
#define __Magic_Sand__Calibration__

#include <iostream>
#include "ofMain.h"
#include "libs/dlib/matrix.h"
#include "libs/dlib/matrix/matrix_qr.h"


class ofxKinectProjectorToolkit
{
public:
    ofxKinectProjectorToolkit(ofVec2f projRes, ofVec2f kinectRes);
    
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
	ofVec2f projRes;
	ofVec2f kinectRes;
};

#endif /* defined(__Magic_Sand__Calibration__) */

