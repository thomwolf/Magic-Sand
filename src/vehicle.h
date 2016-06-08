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
    const ofVec2f& getCurrentForce() const {
        return currentForce;
    }
    
    std::vector<ofVec2f> getForces(void);

private:
    
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