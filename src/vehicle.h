#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "KinectProjector.h"

class Vehicle{

public:
    Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater, ofVec2f motherLocation);
    
    virtual void setup();
    void updateBeachDetection();

    ofPoint seekEffect();
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
    
    void setMotherLocation(ofVec2f loc){
        motherLocation = loc;
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
    int waitCounter;
    int waitTime;
    int maxWaitingTime;
    int minWaitingTime;
    
    bool mother;
    ofVec2f motherLocation;
    
    // For slope effect
    float beachDist;
    ofVec2f beachSlope;
    
    bool liveInWater; // true for fish who want to stay in the water, false for rabbits who want to stay on the ground
    
    ofVec2f projectorCoord;
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
    Fish(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, true, motherLocation){}

    void setup();
    ofPoint wanderEffect();
    //    ofPoint separateEffect(vector<vehicle> vehicles);
    void applyBehaviours(bool seekMother);
    void draw();
};

class Rabbit : public Vehicle {
public:
    Rabbit(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, false, motherLocation){}
    
    void setup();
    ofPoint wanderEffect();
    //    ofPoint separateEffect(vector<vehicle> vehicles);
    void applyBehaviours(bool seekMother);
    void draw();
};

