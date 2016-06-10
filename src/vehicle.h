#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

class vehicle{

public:
    ofPoint location;
  
    void setup(int x, int y, ofRectangle sborders, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution);
    void updateBasePlaneEq(ofVec4f sbasePlaneEq);
    void updateFilteredDepthImageAndGradient(ofxCvFloatImage sFilteredDepthImage, ofVec2f* sgradient);

    ofPoint seekForce(const ofPoint & target);
    ofPoint separateForce(vector<vehicle> vehicles);
    ofPoint bordersForce();
    ofPoint slopesForce();
    void applyForce(const ofPoint & force);

    void applyBehaviours(vector<vehicle> vehicles, ofPoint target);
    void update();
    void draw();

    const ofPoint& getLocation() const {
        return location;
    }
    const ofPoint& getVelocity() const {
        return velocity;
    }
    
    const ofVec2f& getCurrentForce() const {
        return currentForce;
    }
    
    std::vector<ofVec2f> getForces(void);

private:
    //Images and cv matrixes
    ofxCvFloatImage             FilteredDepthImage;
    ofVec2f*                    gradient;
    ofMatrix4x4                 kinectWorldMatrix;
    ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format
    int kinectResX;
    int gradFieldcols, gradFieldrows;
    double gradFieldresolution;

    ofVec2f separateF ;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;

    ofPoint velocity;
    ofPoint acceleration;
    ofVec2f currentForce;
    
//    const ofVec2f gradient;
    ofRectangle borders, internalBorders;
    float topSpeed;
    float maxForce; 
    int r, minborderDist, desiredseparation, cor;
    int screenWidth, screenHeight;
    
    
};
