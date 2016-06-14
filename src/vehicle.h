#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

class vehicle{

public:
    ofPoint location;
    
    float animalCoef; // 1 for fish who want to stay in the water, -1 for rabbits who want to stay on the ground
  
    void setup(int x, int y, ofRectangle sborders);
//    void updateBasePlaneEq(ofVec4f sbasePlaneEq);
    void updateBeachDetection(bool beach, float beachDist, ofVec2f beachSlope);

    ofPoint seekEffect(const ofPoint & target);
    ofPoint separateEffect(vector<vehicle> vehicles);
    ofPoint bordersEffect();
    ofPoint slopesEffect();
    ofPoint wanderEffect();
    void applyVelocityChange(const ofPoint & force);

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
    
    const float getAngle() const {
        return angle;
    }
    
    std::vector<ofVec2f> getForces(void);

private:
    ofVec2f separateF ;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;
    ofVec2f wanderF ;

    ofPoint velocity;
    ofPoint globalVelocityChange;
    ofVec2f currentForce;
    float angle; // direction of the fish drawing
    float wandertheta;
    
    // For slope effect
    bool beach;
    float beachDist;
    ofVec2f beachSlope;
    
//    const ofVec2f gradient;
    ofRectangle borders, internalBorders;
    float topSpeed;
    float maxVelocityChange;
    float maxRotation;
    int r, minborderDist, desiredseparation, cor;
    int screenWidth, screenHeight;
    
    
};
