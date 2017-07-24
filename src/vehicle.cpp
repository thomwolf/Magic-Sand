/***********************************************************************
vehicle.cpp - vehicle class (fish & rabbits moving in the sandbox)
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

#include "vehicle.h"


Vehicle::Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, float sangle) {
    kinectProjector = k;
    location = slocation;
    borders = sborders;
    angle = sangle;
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    wandertheta = 0;
}

void Vehicle::updateBeachDetection(){
    // Find sandbox gradients and elevations in the next 10 steps of vehicle v, update vehicle variables
    ofPoint futureLocation;
    futureLocation = location;
    beachSlope = ofVec2f(0);
    beach = false;
    int i = 1;
    while (i < 10 && !beach)
    {
        bool water = kinectProjector->elevationAtKinectCoord(futureLocation.x, futureLocation.y) < 0;
        if (water)
        {
            beach = true;
            beachDist = i;
            beachSlope = kinectProjector->gradientAtKinectCoord(futureLocation.x,futureLocation.y);
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
}

ofPoint Vehicle::bordersEffect(){
    ofPoint desired, futureLocation;
    
    // Predict location 10 (arbitrary choice) frames ahead
    futureLocation = location + velocity*10;
    
    ofPoint target = location;
    if (!internalBorders.inside(futureLocation)){ // Go to the opposite direction
        border = true;
        if (futureLocation.x < internalBorders.getLeft())
            target.x = borders.getRight();
        if (futureLocation.y < internalBorders.getTop())
            target.y = borders.getBottom();
        if (futureLocation.x > internalBorders.getRight())
            target.x = borders.getLeft();
        if (futureLocation.y > internalBorders.getBottom())
            target.y = borders.getTop();
    } else {
        border = false;
    }
    
    //desired = target - location;
    //desired.normalize();
    //desired *= topSpeed;
    
    ofPoint velocityChange(0);
    //velocityChange = desired - velocity;
    //velocityChange.limit(maxVelocityChange);
    return velocityChange;
}

ofPoint Vehicle::wanderEffect(){
    
    ofPoint velocityChange, desired;

    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofPoint front = velocity;
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
    return velocityChange;
}

// Determine Elevation and return velocity change depending on Height

ofPoint Vehicle::hillEffect() {
	ofPoint velocityChange, futureLocation, currentLocation;
	float futureelevation, currentelevation;

	currentLocation = location;
	futureLocation = location + velocity * 10;
	currentelevation = kinectProjector->elevationAtKinectCoord(currentLocation.x, currentLocation.y);
	futureelevation = kinectProjector->elevationAtKinectCoord(futureLocation.x,futureLocation.y);
	if (currentelevation > futureelevation) {
		velocityChange = ofPoint(-0.5,0,0);
	}
	if (currentelevation < futureelevation) {
		velocityChange = velocity * 2;

	}
	if (currentelevation == futureelevation) {
		//no Velocity Change 
	}
	return velocityChange;
}

ofPoint Vehicle::windEffect(float windspeed, float winddirection) {
	ofPoint velocityChange;
	int directionOfWind = (int)winddirection;
	int inverseWinddirection = (directionOfWind - 180) % 360;
	int directionOfFire = (int)angle;
	int positiveRangeFire = (directionOfFire + 45) % 360;
	int negativeRangeFire = (directionOfFire - 45) % 360;
	int windForce;
	// set Factor for velocityChange
	if (windspeed > 1) {
		windForce = 1.25;
	} else if (windspeed > 4) {
		windForce = 1.5;
	} else if (windspeed > 6) {
		windForce = 1.75;
	} else if (windspeed > 8) {
		windForce = 2;
	} else {
		windForce = 0;
	}

	float windDir = ofDegToRad(winddirection);
	ofPoint desired = ofVec2f(cos(windDir), sin(windDir));
	desired.normalize();
	desired *= windForce;

	return desired.limit(maxVelocityChange);
}


ofPoint Vehicle::slopesEffect(){
    ofPoint desired, velocityChange;
    
    desired = beachSlope;
    desired.normalize();
    desired *= topSpeed;
    if(beach){
        desired /= beachDist; // The closest the beach is, the more we want to avoid it
    }
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);

    return velocityChange;
}

void Vehicle::applyVelocityChange(const ofPoint & velocityChange){
    globalVelocityChange += velocityChange;
}
// Movement: vehicle update rotation ...
void Vehicle::update(){
    projectorCoord = kinectProjector->kinectCoordToProjCoord(location.x, location.y);
    velocity += globalVelocityChange;
    velocity.limit(topSpeed);
    location += velocity;
    globalVelocityChange *= 0;

    float desiredAngle = ofRadToDeg(atan2(velocity.y,velocity.x));
    float angleChange = desiredAngle - angle;
    angleChange += (angleChange > 180) ? -360 : (angleChange < -180) ? 360 : 0; // To take into account that the difference between -180 and 180 is 0 and not 360
    angleChange *= velocity.length();
    angleChange /= topSpeed;
    angleChange = max(min(angleChange, maxRotation), -maxRotation);
    angle += angleChange;
}

//==============================================================
// Derived class Fire
//==============================================================

void Fire::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    
    wanderR = 50;         // Radius for our "wander circle"
    wanderD = 0;         // Distance for our "wander circle"
    change = 1;
    
    r = 12;
    maxVelocityChange = 1;
    maxRotation = 360;
    topSpeed = 1;
    velocityIncreaseStep = 2;
    minVelocity = velocityIncreaseStep;
    
    intensity = 3;
	alive = true;
}
// Rotation vom Rabbit : Simon
ofPoint Fire::wanderEffect(){
    
    ofPoint velocityChange, desired;
    
    wandertheta = ofRandom(-change,change);     // Randomly change wander theta
    
    float currDir = ofDegToRad(angle);
    ofPoint front = ofVec2f(cos(currDir), sin(currDir));
    
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
   // float h = ofradtodeg(atan2(front.x,front.y));
  	//float h = front.angle(ofvec2f(1,0)); // we need to know the heading to offset wandertheta
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+currDir),wanderR*sin(wandertheta+currDir));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired;// - velocity;
    velocityChange.limit(maxVelocityChange);
    
    return velocityChange;
}
// Forces : seekF, bordersF, slopesF, wanderF, ==> Temp, Wind, Humid, ... hier mögl.
void Fire::applyBehaviours() {

}

void Fire::applyBehaviours(float windspeed, float winddirection) {
    updateBeachDetection();
    
	windF = windEffect(windspeed, winddirection);
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
	hillF = hillEffect();
    
    ofPoint littleSlopeF = slopesF;
    

    bordersF *=0.5;
    slopesF *= 2;//2;
    wanderF *= 1;// Used to introduce some randomness in the direction changes
    littleSlopeF *= 1;
    
    float currDir = ofDegToRad(angle);
    ofPoint oldDir = ofVec2f(cos(currDir), sin(currDir));
    oldDir.scale(velocityIncreaseStep);
	ofPoint newDir;
	newDir += wanderF;
	newDir += hillF;
	newDir += windF;

    
	if (beach)
        oldDir.scale(velocityIncreaseStep/beachDist);

	if (!beach && !border)
		{   
		applyVelocityChange(newDir);
		applyVelocityChange(oldDir); // Just accelerate
		} else { // Wee need to decelerate and then change direction
		    if (velocity.lengthSquared() > minVelocity*minVelocity) // We are not stopped yet
		    {
		        applyVelocityChange(-oldDir); // Just deccelerate
				applyVelocityChange(-newDir);
		    }  else {
				// Stops the Fire agent
				velocity = ofPoint(0);
				alive = false;
			}
		}
}

void Fire::draw(){
    if(!alive){
        intensity--;
    }
    // saves the current coordinate system
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    ofColor color = getFlameColor();
    
    float sc = 2;
    
    ofFill();
    
    ofPath flame;
    flame.arc(0,-3*sc,3*sc,3*sc,90,270);
    flame.arc(0,-5*sc,sc,sc,90,270);
    flame.arc(0,-2*sc,2*sc,2*sc,270,90);
    flame.setFillColor(color);
    flame.setStrokeWidth(0);
    flame.draw();
    
    ofNoFill();
    
    // restore the pushed state
    ofPopMatrix();
}

void Fire::kill(){
    alive = false;
}

ofColor Fire::getFlameColor(){
    float intensityFactor = intensity <= 0 ? 0 : intensity*0.33;
    int red = 255 * intensityFactor;
    int green = 64 * intensityFactor;
    int blue = 0 * intensityFactor;
    return ofColor(red, green, blue);
}
