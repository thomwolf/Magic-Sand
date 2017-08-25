/***********************************************************************
vehicle.h - vehicle class (animals/BOIDS moving in the sandbox)
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)

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

#include "../KinectProjector/KinectProjector.h"

// We can not interchange info from Fish to Sharks and from Sharks to Fish at the same time. This class is used as an intermediate
class DangerousBOID
{
public:

	DangerousBOID(ofPoint loc, ofPoint vel, double sz);

	ofPoint location;
	ofPoint velocity;
	double size;
};


// A vehicle is basically a BOID. 
/*
The seminal paper on BOIDS can be found here : http://www.red3d.com/cwr/papers/1987/boids.html
A good tutorial is here : https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732  */
class Vehicle{

public:
    Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater, ofVec2f motherLocation);
    
    // Virtual functions
    virtual void setup() = 0;
 //   virtual void applyBehaviours(bool seekMother, std::vector<Vehicle> vehicles) = 0;
    virtual void draw() = 0;
    
    void update();
    
    std::vector<ofVec2f> getForces(void);
    
    const ofPoint& getLocation() const {
        return location;
    }

	const double getSize() const
	{
		return size;
	}
	void setSizeAndSpeed(double sz);


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

	static void setDrawFlipped(bool df)
	{
		DrawFlipped = df;
	};
    
protected:
    void updateBeachDetection();
    ofPoint seekMotherEffect();
	ofPoint arrivalEffect(ofPoint target);
		
	ofPoint bordersEffect();
    ofPoint slopesEffect();
    virtual ofPoint wanderEffect();
    void applyVelocityChange(const ofPoint & force);
    
    std::shared_ptr<KinectProjector> kinectProjector;

	int GetTimeStamp();

    ofPoint location;
    ofPoint velocity;
    ofPoint globalVelocityChange;
    ofVec2f currentForce;
    float angle; // direction of the drawing
	float size; // Size of vehicle

    ofVec2f separateF ;
	ofVec2f alignF;
	ofVec2f cohesionF;
    ofVec2f seekF ;
    ofVec2f bordersF ;
    ofVec2f slopesF ;
    ofVec2f wanderF ;
	ofVec2f fleeF;

    bool beach;
    bool border;
	bool isFleeing;

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
//	int r;
	int minborderDist;
	int desiredseparation;
//	int cor;
    
    float wanderR ;         // Radius for our "wander circle"
    float wanderD ;         // Distance for our "wander circle"
    float change ;
    float wandertheta;
    float topSpeed;

	int spawnTime;
	int maxAge;
	int DeathAge;

	// Should the vehicles been drawn flipped
	// This variable is shared among all instances 
	static bool DrawFlipped;
};

class Fish : public Vehicle {
public:
    Fish(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, true, motherLocation){}

    void setup();
	void UpdateAgeAndSize();
	int getCurrentAge();
	void reSpawn(vector<Fish>& vehicles);
	void applyBehaviours(bool seekMother, std::vector<Fish>& vehicles, std::vector<DangerousBOID>& dangers);
    void draw();
    
	void setSizeAndSpeed(double sz);

private:

	ofPoint separateEffect(std::vector<Fish>& vehicles);
	ofPoint alignEffect(std::vector<Fish>& vehicles);
	ofPoint cohesionEffect(std::vector<Fish>& vehicles);
		
    ofPoint wanderEffect();
	ofPoint fleeEffect(std::vector<DangerousBOID>& dangers);

	bool isOldest;
};

class Shark : public Vehicle {
public:
	Shark(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, true, motherLocation) {}

	void setup();
	void applyBehaviours(std::vector<Fish>& vehicles);
	void locatePray(std::vector<Fish>& vehicles);
	
	void draw();

private:

	//ofPoint separateEffect(std::vector<Fish>& vehicles);
	//ofPoint alignEffect(std::vector<Fish>& vehicles);
	//ofPoint cohesionEffect(std::vector<Fish>& vehicles);

	int hunger;
	int timeAtLastMeal;
	bool isHunting;
	int timeAtHuntStart;
	int prayID;

	ofVec2f huntF;

	ofPoint wanderEffect();
	ofPoint huntEffect(std::vector<Fish>& vehicles);
	void setSizeAndSpeed(double sz);
	ofPoint locateDensestFishPopulation(std::vector<Fish>& vehicles);
	void reSpawn(std::vector<Fish>& vehicles);
};



class Rabbit : public Vehicle {
public:
    Rabbit(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, ofVec2f motherLocation) : Vehicle(k, slocation, sborders, false, motherLocation){}
    
    void setup();
    void applyBehaviours(bool seekMother, std::vector<Rabbit>& vehicles);
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

