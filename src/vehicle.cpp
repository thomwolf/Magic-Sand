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

Vehicle::Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, bool sliveInWater, ofVec2f smotherLocation) {
    kinectProjector = k;
    liveInWater = sliveInWater;
    location = slocation;
    borders = sborders;
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    wandertheta = 0;
    mother = false;
    motherLocation = smotherLocation;
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
        bool overwater = kinectProjector->elevationAtKinectCoord(futureLocation.x, futureLocation.y) > 0;
        if ((overwater && liveInWater) || (!overwater && !liveInWater))
        {
            beach = true;
            beachDist = i;
            beachSlope = kinectProjector->gradientAtKinectCoord(futureLocation.x,futureLocation.y);
            if (liveInWater)
                beachSlope *= -1;
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
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    ofPoint velocityChange(0);
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
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

ofPoint Vehicle::seekEffect(){
    ofPoint desired;
    desired = motherLocation - location;
    
    float d = desired.length();
    desired.normalize();
    
    //If we are closer than XX pixels slow down
    if (d < 10) {
        desired *= ofMap(d,0,100,0,topSpeed);
        mother = true;
    } else {
        //Otherwise, proceed at maximum speed.
        desired *= topSpeed;
    }
    
    ofPoint velocityChange;
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
    
    //If we are further than XX pixels we don't see the mother
    if (d > 100) {
        velocityChange = ofPoint(0);
    }
    
    return velocityChange;
}

//--------------------------------------------------------------
//ofPoint Vehicle::separateEffect(vector<vehicle> vehicles){
////    float desiredseparation = r*2;
//    ofPoint velocityChange;
//    int count = 0;
//    ofPoint diff;
//    vector<vehicle>::iterator other;
//    for (other = vehicles.begin(); other < vehicles.end(); other++){
//        float d = (location - other->getLocation()).length();
//        if((d>0) && (d < desiredseparation)){
//            diff = location - other->getLocation();
//            diff.normalize();
//            diff /= d;
//            velocityChange+= diff;
//            count ++;
//        }
//    }
//    if(count > 0){
//        velocityChange /= count;
//        velocityChange.normalize();
//        velocityChange*=topSpeed;
//
//        velocityChange -= velocity;
//        velocityChange.limit(maxVelocityChange);
//    }
//    return velocityChange;
//}

std::vector<ofVec2f> Vehicle::getForces(void)
{
    std::vector<ofVec2f> Forces;
    Forces.push_back( separateF);
    Forces.push_back( seekF);
    Forces.push_back( bordersF);
    Forces.push_back( slopesF);
    Forces.push_back( wanderF);
    return Forces;
}

void Vehicle::applyVelocityChange(const ofPoint & velocityChange){
    globalVelocityChange += velocityChange;
}

void Vehicle::update(){
    projectorCoord = kinectProjector->kinectCoordToProjCoord(location.x, location.y);
    if (!mother || velocity.lengthSquared() != 0)
    {
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
}


//==============================================================
// Derived class Fish
//==============================================================

void Fish::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    
    wanderR = 10;         // Radius for our "wander circle"
    wanderD = 80;         // Distance for our "wander circle"
    change = 0.3;
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 30;
    topSpeed =2;
}

ofPoint Fish::wanderEffect(){
    
    ofPoint velocityChange, desired;
    
    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofPoint front = velocity;
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
    float h = ofRadToDeg(atan2(front.y,front.x)); // Signed angle
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
    
    return velocityChange;
}

void Fish::applyBehaviours(bool seekMother){
    updateBeachDetection();
    
    //    separateF = separateEffect(vehicles);
    seekF = ofVec2f(0);
    if (seekMother)
        seekF = seekEffect();
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
    
    //    separateF*=1;//2;
    seekF *= 1;
    bordersF *=2;
    slopesF *= 2;//2;
    wanderF *= 0.8;
    
    if (beach){
        applyVelocityChange(slopesF);
    }
    if (border){
        applyVelocityChange(bordersF);
    }
    //    applyVelocityChange(separateF);
    if (seekF.lengthSquared() == 0){
        applyVelocityChange(wanderF);
    } else {
        applyVelocityChange(seekF);
    }
    //    currentForce = separateF+seekF+bordersF+slopesF;
}

void Fish::draw()
{
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    
    // Compute tail angle
    float nv = 0.5;//velocity.lengthSquared()/10; // Tail movement amplitude
    float fact = 50+250*velocity.length()/topSpeed;
    float tailangle = nv/25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50)-25);
    
    // Color of the fish
    nv = 255;
    fact = 50;
    float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
    
    // Fish scale
    float sc = 7;
    float tailSize = 1*sc;
    float fishLength = 2*sc;
    float fishHead = tailSize;
    
    ofPolyline fish;
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8)));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle+0.8), tailSize*sin(tailangle+0.8)));
    fish.curveTo( ofPoint(-fishLength, 0));
    fish.curveTo( ofPoint(0, -fishHead));
    fish.curveTo( ofPoint(fishHead, 0));
    fish.curveTo( ofPoint(0, fishHead));
    fish.curveTo( ofPoint(-fishLength, 0));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8)));
    fish.curveTo( ofPoint(-fishLength-tailSize*cos(tailangle-0.8), tailSize*sin(tailangle-0.8)));
    fish.close();
    ofSetLineWidth(2.0);
    ofColor c = ofColor(255);
    ofSetColor(c);
    if (mother)
    {
        c.setHsb((int)hsb, 255, 255); // rainbow
        ofFill();
    } else {
        ofNoFill();
    }
    fish.draw();
    if (mother)
    {
        c.setHsb(255-(int)hsb, 255, 255); // rainbow
        ofSetColor(c);
    }
    ofDrawCircle(0, 0, sc*0.5);
    ofNoFill();
    ofPopMatrix();
}

//==============================================================
// Derived class Rabbit
//==============================================================

void Rabbit::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    
    wanderR = 50;         // Radius for our "wander circle"
    wanderD = 0;         // Distance for our "wander circle"
    change = 1;
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 360;
    topSpeed =3;
    velocityIncreaseStep = 2;
    maxStraightPath = 20;
    minVelocity = velocityIncreaseStep;
    
    minWaitingTime = 2;
    maxWaitingTime = 10;
    setWait = false;
}

ofPoint Rabbit::wanderEffect(){
    
    ofPoint velocityChange, desired;
    
    wandertheta = ofRandom(-change,change);     // Randomly change wander theta
    
    float currDir = ofDegToRad(angle);
    ofPoint front = ofVec2f(cos(currDir), sin(currDir));
    
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
    //    float h = ofRadToDeg(atan2(front.x,front.y));
    //	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+currDir),wanderR*sin(wandertheta+currDir));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired;// - velocity;
    velocityChange.limit(maxVelocityChange);
    
    return velocityChange;
}

void Rabbit::applyBehaviours(bool seekMother){
    updateBeachDetection();
    
    //    separateF = separateEffect(vehicles);
    seekF = ofVec2f(0);
    if (seekMother)
        seekF = seekEffect();
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
    
    ofPoint littleSlopeF = slopesF;
    
    //    separateF*=1;//2;
    seekF *= 1;
    bordersF *=0.5;
    slopesF *= 2;//2;
    wanderF *= 1;// Used to introduce some randomness in the direction changes
    littleSlopeF *= 1;
    
    float currDir = ofDegToRad(angle);
    ofPoint oldDir = ofVec2f(cos(currDir), sin(currDir));
    oldDir.scale(velocityIncreaseStep);
    if (beach)
        oldDir.scale(velocityIncreaseStep/beachDist);
    
    if (setWait){
        waitCounter++;
        if (waitCounter > waitTime){
            setWait = false;
            // Compute a new direction
            //            oldDir.scale(topSpeed);
            wanderF = wanderEffect();
            ofPoint newDir;
            if (seekF.lengthSquared() == 0){
                newDir = wanderF;
            } else {
                newDir = seekF;
            }

            //            newDir +=littleSlopeF;
            if (border)
                newDir +=bordersF;
            if (beach)
                newDir +=slopesF;
            
            newDir.scale(velocityIncreaseStep);
            applyVelocityChange(newDir);
            
            currentStraightPathLength = 0;
            angle = ofRadToDeg(atan2(newDir.y,newDir.x));
            
        }
    } else {
        if (!beach && !border && !mother && currentStraightPathLength < maxStraightPath)
        {
            
            applyVelocityChange(oldDir); // Just accelerate
            currentStraightPathLength++;
        } else { // Wee need to decelerate and then change direction
            if (velocity.lengthSquared() > minVelocity*minVelocity) // We are not stopped yet
            {
                applyVelocityChange(-oldDir); // Just deccelerate
            } else {
                velocity = ofPoint(0);
                setWait = true;
                waitCounter = 0;
                waitTime = ofRandom(minWaitingTime, maxWaitingTime);
                if (beach)
                    waitTime = 0;
            }
        }
    }
}

void Rabbit::draw()//, std::vector<ofVec2f> forces)
{
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    
    // Rabbit scale
    float sc = 1;
    
    ofFill();
    ofSetLineWidth(1.0);  // Line widths apply to polylines
    
    ofColor c1 = ofColor(255);
    ofColor c2 = ofColor(0);
    if (mother)
    {
        float nv = 255;
        int fact = 50;
        float et = ofGetElapsedTimef();
        float hsb = nv/50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));
        c1.setHsb((int)hsb, 255, 255); // rainbow
        c2.setHsb(255-(int)hsb, 255, 255);
    }
    
    ofPath body;
    body.curveTo( ofPoint(-2*sc, 5.5*sc));
    body.curveTo( ofPoint(-2*sc, 5.5*sc));
    body.curveTo( ofPoint(-9*sc, 7.5*sc));
    body.curveTo( ofPoint(-17*sc, 0*sc));
    body.curveTo( ofPoint(-9*sc, -7.5*sc));
    body.curveTo( ofPoint(-2*sc, -5.5*sc));
    body.curveTo( ofPoint(4*sc, 0*sc));
    body.curveTo( ofPoint(4*sc, 0*sc));
    body.close();
    ofSetColor(c1);
    body.setFillColor(c1);
    body.draw();
    
    ofSetColor(c2);
    ofDrawCircle(-19*sc, 0, 2*sc);

    ofPath head;
    head.curveTo( ofPoint(0, 1.5*sc));
    head.curveTo( ofPoint(0, 1.5*sc));
    head.curveTo( ofPoint(-3*sc, 1.5*sc));
    head.curveTo( ofPoint(-9*sc, 3.5*sc));
    head.curveTo( ofPoint(0, 5.5*sc));
    head.curveTo( ofPoint(8*sc, 0));
    head.curveTo( ofPoint(0, -5.5*sc));
    head.curveTo( ofPoint(-9*sc, -3.5*sc));
    head.curveTo( ofPoint(-3*sc, -1.5*sc));
    head.curveTo( ofPoint(0, -1.5*sc));
    head.curveTo( ofPoint(0, -1.5*sc));
    head.close();
    ofSetColor(c2);
    head.setFillColor(c2);
    head.draw();
    
    ofSetColor(c1);
    ofDrawCircle(8.5*sc, 0, 1*sc);

    ofSetColor(255);
    ofNoFill();

    ofPopMatrix();
}


