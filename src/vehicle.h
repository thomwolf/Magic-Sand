#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "KinectProjector.h"

class Vehicle{

public:
    Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater);
    
    virtual void setup();
    void updateBeachDetection();

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

protected:
    std::shared_ptr<KinectProjector> kinectProjector;

    ofVec2f separateF ;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;
    ofVec2f wanderF ;

    ofPoint location;
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
    
    bool liveInWater; // true for fish who want to stay in the water, false for rabbits who want to stay on the ground
    
//    const ofVec2f gradient;
    ofRectangle borders, internalBorders;
//    float topSpeed;
    float maxVelocityChange;
    float maxRotation;
    int r, minborderDist, desiredseparation, cor;
    
    float wanderR ;         // Radius for our "wander circle"
    float wanderD ;         // Distance for our "wander circle"
    float change ;
    float wandertheta;
    float topSpeed;
};

class Fish : public Vehicle {
public:
    Fish(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater) : Vehicle(k, slocation, sborders, sliveInWater){}

    void setup();
    ofPoint wanderEffect();
    //    ofPoint separateEffect(vector<vehicle> vehicles);
    void applyBehaviours(ofPoint target);
    void draw();
};

class Rabbit : public Vehicle {
public:
    Rabbit(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater) : Vehicle(k, slocation, sborders, sliveInWater){}
    
    void setup();
    ofPoint wanderEffect();
    //    ofPoint separateEffect(vector<vehicle> vehicles);
    void applyBehaviours(ofPoint target);
    void draw();
};

