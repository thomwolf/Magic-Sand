#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

class vehicle{

public:
    ofPoint location;
    
    float animalCoef; // 1 for fish who want to stay in the water, -1 for rabbits who want to stay on the ground
  
    virtual void setup(int x, int y, ofRectangle sborders);
//    void updateBasePlaneEq(ofVec4f sbasePlaneEq);
    void updateBeachDetection(bool beach, float beachDist, ofVec2f beachSlope);

    ofPoint seekEffect(const ofPoint & target);
    ofPoint bordersEffect();
    ofPoint slopesEffect();
    virtual ofPoint wanderEffect();
    void applyVelocityChange(const ofPoint & force);

    void update();
    void draw();
    
//    ofPoint separateEffect(vector<vehicle> vehicles);
//    virtual void applyBehaviours(vector<vehicle> vehicles, ofPoint target);

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
    
    const bool foundMother() const {
        return mother;
    }
    
    std::vector<ofVec2f> getForces(void);

    float wanderR ;         // Radius for our "wander circle"
    float wanderD ;         // Distance for our "wander circle"
    float change ;
    float wandertheta;
    float topSpeed;

protected:
    ofVec2f separateF ;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;
    ofVec2f wanderF ;

    ofPoint velocity;
    ofPoint globalVelocityChange;
    ofVec2f currentForce;
    float angle; // direction of the drawing
    float velocityIncreaseStep; // Rabbit increase step
    float minVelocity;
    int maxStraightPath; // max rabbit straight path length
    int currentStraightPathLength;// current rabbit straight path length
    
    bool beach;
    bool border;
    bool setWait;
    bool mother;
    int waitCounter, waitTime, maxWaitingTime, minWaitingTime;
    // For slope effect
    float beachDist;
    ofVec2f beachSlope;
    
//    const ofVec2f gradient;
    ofRectangle borders, internalBorders;
//    float topSpeed;
    float maxVelocityChange;
    float maxRotation;
    int r, minborderDist, desiredseparation, cor;
    int screenWidth, screenHeight;
    
    
};

class Fish : public vehicle {
public:
//    ofPoint separateEffect(vector<vehicle> vehicles);
    void setup(int x, int y, ofRectangle sborders);
    ofPoint wanderEffect();

    void applyBehaviours(ofPoint target);
};

class Rabbit : public vehicle {
public:
 //   ofPoint separateEffect(vector<vehicle> vehicles);
    void setup(int x, int y, ofRectangle sborders);
    ofPoint wanderEffect();

    void applyBehaviours(ofPoint target);
};

