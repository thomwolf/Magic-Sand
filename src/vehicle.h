/***********************************************************************
vehicle.h - vehicle class (fish & rabbits moving in the sandbox)
Copyright (c) 2016 Thomas Wolf

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
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "KinectProjector/KinectProjector.h"

class Vehicle{

public:
    Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater, ofVec2f motherLocation);
    
    // Virtual functions
    virtual void setup() = 0;
    virtual void applyBehaviours(bool seekMother) = 0;
    virtual void draw() = 0;
    
    void update();
    
    std::vector<ofVec2f> getForces(void);
    
    const ofPoint& getLocation() const {
        return location;
    }
    const ofPoint& getVelocity() const {
        return velocity;
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
    
protected:
    void updateBeachDetection();
    ofPoint seekEffect();
    ofPoint bordersEffect();
    ofPoint slopesEffect();
    virtual ofPoint wanderEffect();
    void applyVelocityChange(const ofPoint & force);
    
    std::shared_ptr<KinectProjector> kinectProjector;

    ofPoint location;
    ofPoint velocity;
    ofPoint globalVelocityChange;
    ofVec2f currentForce;
    float angle; // direction of the drawing
    
    ofVec2f separateF ;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;
    ofVec2f wanderF ;

    bool beach;
    bool border;
    
    bool mother;
    ofVec2f motherLocation;
    
    // For slope effect
    float beachDist;
    ofVec2f beachSlope;
    
    bool liveInWater; // true for fish who want to stay in the water, false for rabbits who want to stay on the ground
    
    ofVec2f projectorCoord;
    ofRectangle borders, internalBorders;
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
    void applyBehaviours(bool seekMother);
    void draw();
    
private:
    ofPoint wanderEffect();
};

class Rabbit : public Vehicle {
public:
    Rabbit(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, false, motherLocation){}
    
    void setup();
    void applyBehaviours(bool seekMother);
    void draw();

private:
    ofPoint wanderEffect();

    int maxStraightPath; // max rabbit straight path length
    int currentStraightPathLength;// current rabbit straight path length
    
    float velocityIncreaseStep; // Rabbit increase step
    float minVelocity;
    bool setWait;
    int waitCounter;
    int waitTime;
    int maxWaitingTime;
    int minWaitingTime;
};

