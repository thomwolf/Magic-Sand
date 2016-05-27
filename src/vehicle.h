#pragma once
#include "ofMain.h"

class vehicle{

public:
    ofPoint location;
  
    void setup(int x, int y, ofRectangle sborders);

    ofPoint seekForce(const ofPoint & target);
    ofPoint separateForce(vector<vehicle> vehicles);
    ofPoint bordersForce();
    ofPoint slopesForce(ofVec2f* gradient);
    void applyForce(const ofPoint & force);

    void applyBehaviours(vector<vehicle> vehicles, ofVec2f* gradient, ofPoint target);
    void update();
    void draw();

    const ofPoint& getLocation() const {
        return location;
    }
    
private:
    
    ofPoint velocity;
    ofPoint acceleration;
    
//    const ofVec2f gradient;
    ofRectangle borders, internalBorders;
    float topSpeed;
    float maxForce; 
    int r, minborderDist, desiredseparation, cor;
    int screenWidth, screenHeight;
    
    
};