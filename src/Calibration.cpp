//
//  Calibration.cpp
//  Magic Sand
//
//

#include "Calibration.h"

ofxKinectProjectorToolkit::ofxKinectProjectorToolkit() {
    calibrated = false;
}

void ofxKinectProjectorToolkit::calibrate(vector<ofVec3f> pairsKinect,
                                          vector<ofVec2f> pairsProjector) {
    int nPairs = pairsKinect.size();
    A.set_size(nPairs*2, 11);
    y.set_size(nPairs*2, 1);
    
    for (int i=0; i<nPairs; i++) {
        A(2*i, 0) = pairsKinect[i].x;
        A(2*i, 1) = pairsKinect[i].y;
        A(2*i, 2) = pairsKinect[i].z;
        A(2*i, 3) = 1;
        A(2*i, 4) = 0;
        A(2*i, 5) = 0;
        A(2*i, 6) = 0;
        A(2*i, 7) = 0;
        A(2*i, 8) = -pairsKinect[i].x * pairsProjector[i].x;
        A(2*i, 9) = -pairsKinect[i].y * pairsProjector[i].x;
        A(2*i, 10) = -pairsKinect[i].z * pairsProjector[i].x;
        
        A(2*i+1, 0) = 0;
        A(2*i+1, 1) = 0;
        A(2*i+1, 2) = 0;
        A(2*i+1, 3) = 0;
        A(2*i+1, 4) = pairsKinect[i].x;
        A(2*i+1, 5) = pairsKinect[i].y;
        A(2*i+1, 6) = pairsKinect[i].z;
        A(2*i+1, 7) = 1;
        A(2*i+1, 8) = -pairsKinect[i].x * pairsProjector[i].y;
        A(2*i+1, 9) = -pairsKinect[i].y * pairsProjector[i].y;
        A(2*i+1, 10) = -pairsKinect[i].z * pairsProjector[i].y;
        
        y(2*i, 0) = pairsProjector[i].x;
        y(2*i+1, 0) = pairsProjector[i].y;
    }
    
    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    x = qrd.solve(y);
    cout << "x: "<< x << endl;
    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
    calibrated = true;
}

ofMatrix4x4 ofxKinectProjectorToolkit::getProjectionMatrix() {
    return projMatrice;
}

ofVec2f ofxKinectProjectorToolkit::getProjectedPoint(ofVec3f worldPoint) {
    ofVec4f pts = ofVec4f(worldPoint);
    pts.w = 1;
    ofVec4f rst = projMatrice*(pts);
    ofVec2f projectedPoint(rst.x/rst.z, rst.y/rst.z);
    return projectedPoint;
}

vector<double> ofxKinectProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxKinectProjectorToolkit::loadCalibration(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
    xml.setTo("CALIBRATION");
    for (int i=0; i<11; i++) {
        x(i, 0) = xml.getValue<float>("COEFF"+ofToString(i));
    }
    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 0);
    calibrated = true;
    return true;
}

bool ofxKinectProjectorToolkit::saveCalibration(string path){
    ofXml xml;
    xml.addChild("CALIBRATION");
    xml.setTo("CALIBRATION");
    for (int i=0; i<11; i++) {
        ofXml coeff;
        coeff.addValue("COEFF"+ofToString(i), x(i, 0));
        xml.addXml(coeff);
    }
    xml.setToParent();
    return xml.save(path);
}


