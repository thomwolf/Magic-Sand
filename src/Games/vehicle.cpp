/***********************************************************************
vehicle.cpp - vehicle class (animals/BOIDS moving in the sandbox)
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

#include "vehicle.h"

// Default value of static variable
bool Vehicle::DrawFlipped = false;

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
	spawnTime = GetTimeStamp();
	isFleeing = false;
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

ofPoint Vehicle::seekMotherEffect(){
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


ofPoint Vehicle::arrivalEffect(ofPoint target)
{
	ofPoint desired;
	desired = target - location;

	float d = desired.length();
	desired.normalize();

	//If we are closer than XX pixels slow down
	if (d < 10) {
		desired *= ofMap(d, 0, 100, 0, topSpeed);
	}
	else {
		//Otherwise, proceed at maximum speed.
		desired *= topSpeed;
	}

	ofPoint velocityChange;
	velocityChange = desired - velocity;
	velocityChange.limit(maxVelocityChange);

	return velocityChange;
}

std::vector<ofVec2f> Vehicle::getForces(void)
{
    std::vector<ofVec2f> Forces;
    Forces.push_back( separateF);
	Forces.push_back(alignF);
	Forces.push_back(cohesionF);
    Forces.push_back( seekF);
    Forces.push_back( bordersF);
    Forces.push_back( slopesF);
    Forces.push_back( wanderF);
    return Forces;
}

void Vehicle::setSizeAndSpeed(double sz)
{
	size = sz;
}

void Vehicle::applyVelocityChange(const ofPoint & velocityChange){
    globalVelocityChange += velocityChange;
}

int Vehicle::GetTimeStamp()
{
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	return now->tm_sec + now->tm_min * 60 + now->tm_hour * 3600;
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
    
//    r = 12;
    desiredseparation = 10;
   // maxVelocityChange = 1;
    maxRotation = 30;
	isOldest = false;
	maxAge = 120; // Max two minutes lifetime
	
	// Go back in time to create fish that are big at start
	spawnTime = GetTimeStamp() - ofRandom(maxAge);
	DeathAge = maxAge / 4 + ofRandom(3 * maxAge / 4);
	UpdateAgeAndSize();
}

void Fish::UpdateAgeAndSize()
{
	int currentAge = getCurrentAge();
	float minSize = 1;
	float maxSize = 6;
	double sz = ofMap(currentAge, 0, maxAge, minSize, maxSize);
	setSizeAndSpeed(sz);
}

int Fish::getCurrentAge()
{
	return GetTimeStamp() - spawnTime;
}

void Fish::reSpawn(vector<Fish>& vehicles)
{
	// Do not find your self as the oldest
	spawnTime = GetTimeStamp();

	// Find oldest fish
	int idx = -1;
	int oldestAge = -1;
	for (int i = 0; i < vehicles.size(); i++)
	{
		vehicles[i].isOldest = false;
		if (vehicles[i].getCurrentAge() > oldestAge)
		{
			idx = i;
			oldestAge = vehicles[i].getCurrentAge();
		}
	}
	vehicles[idx].isOldest = true;

	// Spawn right in mother
	location = vehicles[idx].getLocation();

	// Go back in time to create fish that are big at start
	spawnTime = GetTimeStamp() - ofRandom(maxAge / 10);
	DeathAge =  maxAge / 4 + ofRandom(3 * maxAge/4);
	UpdateAgeAndSize();
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


ofPoint Fish::fleeEffect(std::vector<DangerousBOID>& dangers)
{
	ofPoint SumVelocityChange;
	isFleeing = false;
	for (int i = 0; i < dangers.size(); i++)
	{
		ofPoint target = dangers[i].location;

		ofPoint desired;
		desired = location - target;

		float d = desired.length();

		if (d < dangers[i].size)
		{
			desired.normalize();

			desired *= topSpeed;

			ofPoint velocityChange = desired - velocity;

			SumVelocityChange += velocityChange;
			isFleeing = true;
		}
	}
	SumVelocityChange.limit(maxVelocityChange);
	return SumVelocityChange;
}


ofPoint Fish::separateEffect(vector<Fish>& vehicles)
{
	//    float desiredseparation = r*2;
	ofPoint velocityChange;
	int count = 0;
	ofPoint diff;
	vector<Fish>::iterator other;
	for (other = vehicles.begin(); other < vehicles.end(); other++)
	{
		desiredseparation = (size + other->size) * 1.5;

		float d = (location - other->getLocation()).length();
		if ((d > 0) && (d < desiredseparation)) {
			diff = location - other->getLocation();
			diff.normalize();
			diff /= d;
			velocityChange += diff;
			count++;
		}
	}
	if (count > 0) {
		velocityChange /= count;
		velocityChange.normalize();
		velocityChange *= topSpeed;

		velocityChange -= velocity;
		velocityChange.limit(maxVelocityChange);
	}
	return velocityChange;
}

//// For every nearby boid in the system, calculate the average velocity
ofPoint Fish::alignEffect(std::vector<Fish>& vehicles)
{
	//float neighbordist = 25;
	float neighbordist = 10 + size;
	ofPoint velocityChange;

	int count = 0;
	vector<Fish>::iterator other;
	for (other = vehicles.begin(); other < vehicles.end(); other++)
	{
		float d = (location - other->getLocation()).length();

		if ((d > 0) && (d < neighbordist)) 
		{
			velocityChange += other->velocity;
			count++;
		}
	}
	if (count > 0) {
		velocityChange /= count;
		velocityChange.normalize();
		velocityChange *= topSpeed;

		velocityChange -= velocity;
		velocityChange.limit(maxVelocityChange);
	}
	return velocityChange;
}


ofPoint Fish::cohesionEffect(std::vector<Fish>& vehicles)
{
	//float neighbordist = 25;
	float neighbordist = 10 + size;
	ofPoint velocityChange;
	ofPoint desired;

	int count = 0;
	vector<Fish>::iterator other;
	for (other = vehicles.begin(); other < vehicles.end(); other++)
	{
		float d = (location - other->getLocation()).length();

		if ((d > 0) && (d < neighbordist))
		{
			desired += other->location;
			count++;
		}
	}
	if (count > 0) {
		desired /= count;
		velocityChange = desired - location;
		velocityChange.normalize();
		velocityChange *= topSpeed;

		velocityChange -= velocity;
		velocityChange.limit(maxVelocityChange);
	}
	return velocityChange;
}


void Fish::applyBehaviours(bool seekMother, std::vector<Fish>& vehicles, std::vector<DangerousBOID>& dangers){
	int currentAge = getCurrentAge();
	if (currentAge > DeathAge)
	{
		reSpawn(vehicles);
	}
	UpdateAgeAndSize();
	updateBeachDetection();

	alignF = 0.5 * alignEffect(vehicles);
	cohesionF = 0.2 * cohesionEffect(vehicles);
	separateF = separateEffect(vehicles);
    seekF = ofVec2f(0);
    if (seekMother)
        seekF = seekMotherEffect();
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();

	if (dangers.size() > 0)
	{
		fleeF = fleeEffect(dangers);
	}
    
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
    applyVelocityChange(separateF);
	applyVelocityChange(alignF);
	applyVelocityChange(cohesionF);
	applyVelocityChange(fleeF);

    if (seekF.lengthSquared() == 0){
        applyVelocityChange(wanderF);
    } else {
        applyVelocityChange(seekF);
    }
  
}



void Fish::draw()
{
    ofPushMatrix();
    ofTranslate(projectorCoord);
	if (DrawFlipped)
		ofRotate(180+angle);
	else
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
    float sc = size;
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
    ofColor c = ofColor(180,180,180);
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
	if (DeathAge - getCurrentAge() < 10)
	{
		// soon dying
		c = ofColor(128, 128, 128);
		ofSetColor(c);
		ofFill();
	}
	if (isOldest)
	{
		c = ofColor(255, 192, 203);
		ofSetColor(c);
		ofFill();
	}
	if (isFleeing)
	{
		c = ofColor(255, 0, 0);
		ofSetColor(c);
		ofFill();
	}

	ofDrawCircle(0, 0, sc*0.5);
    ofNoFill();
    ofPopMatrix();
}

void Fish::setSizeAndSpeed(double sz)
{
	size = sz;
	topSpeed = size / 4;
	maxVelocityChange = size / 8;

	if (isFleeing)
	{
		topSpeed *= 1.5;
		maxVelocityChange *= 1.5;
	}
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
    
//    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 360;
    topSpeed = 1;
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

void Rabbit::applyBehaviours(bool seekMother, std::vector<Rabbit>& vehicles){
    updateBeachDetection();
    
    //    separateF = separateEffect(vehicles);
    seekF = ofVec2f(0);
    if (seekMother)
        seekF = seekMotherEffect();
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
	if (DrawFlipped)
		ofRotate(180 + angle);
	else
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


void Shark::setup()
{
	minborderDist = 50;
	internalBorders = borders;
	internalBorders.scaleFromCenter((borders.width - minborderDist) / borders.width, (borders.height - minborderDist) / borders.height);

	wanderR = 10;         // Radius for our "wander circle"
	wanderD = 80;         // Distance for our "wander circle"
	change = 0.3;

	desiredseparation = 10;
	maxRotation = 30;
	float minSize = 6;
	float maxSize = 10;
	size = ofRandom(minSize, maxSize);
	hunger = 0;
	isHunting = false;
	prayID = -1;
	timeAtLastMeal = GetTimeStamp();
	setSizeAndSpeed(size);
}


void Shark::locatePray(std::vector<Fish>& vehicles)
{
	int huntRadius = size * 3;

	prayID = -1;
	int praySize = 3; // Minimum size to hunt 
	for (int i = 0; i < vehicles.size(); i++)
	{
		float d = (location - vehicles[i].getLocation()).length();
		if ((d > 0) && (d < huntRadius)) 
		{
			if (vehicles[i].getSize() > praySize)
			{
				praySize = vehicles[i].getSize();
				prayID = i;
			}
		}
	}
	if (prayID != -1)
	{
		isHunting = true;
		timeAtHuntStart = GetTimeStamp();
	}
}

//// Same as pursuit
// https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-pursuit-and-evade--gamedev-2946
ofPoint Shark::huntEffect(std::vector<Fish>& vehicles)
{
	if (prayID >= vehicles.size() || prayID == -1 || GetTimeStamp() - timeAtHuntStart > 10)
	{
		// Something happened to the target or you have been hunting for too long
		prayID = -1;
		isHunting = false;
		return ofPoint(0, 0);
	}

	ofPoint targetLoc = vehicles[prayID].getLocation();
	ofPoint targetVel = vehicles[prayID].getVelocity();
	float targetSize = vehicles[prayID].getSize();

	float d = (location - targetLoc).length();
	if (d < (targetSize+size) / 2)
	{
		// eat the fish!
		hunger -= targetSize;
		// currently just make the Pray fish very small - for testing
//		vehicles[prayID].setSize(2);
		vehicles[prayID].reSpawn(vehicles);

		if (hunger < 0)
		{
			hunger = 0;
			isHunting = 0;
			timeAtLastMeal = GetTimeStamp();
		}
		else
		{
			locatePray(vehicles);
		}

		return ofPoint(0, 0);
	}

	float T = d / topSpeed;
	ofPoint futureLoc = targetLoc + targetVel * T;

	return arrivalEffect(futureLoc);
}

void Shark::setSizeAndSpeed(double sz)
{
	size = sz;

	topSpeed = size / 4;
	maxVelocityChange = size / 8;

	// Chill speed
	if (!isHunting)
	{
		topSpeed = size / 12;
		maxVelocityChange = size / 20;
	}
}

// Very expensive N^2 - could be speeded up using a spatial search structure
ofPoint Shark::locateDensestFishPopulation(std::vector<Fish>& vehicles)
{
	double searchRad = 30;

	double maxDens = -1;
	int maxIdx = -1;
	for (int i = 0; i < vehicles.size(); i++)
	{
		double density = 0;
		for (int j = 0; j < vehicles.size(); j++)
		{
			if (i != j)
			{
				float d = (vehicles[i].getLocation() - vehicles[j].getLocation()).length();
				if (d < searchRad)
				{
					density += vehicles[j].getSize();
				}
			}
		}
		if (density > maxDens)
		{
			maxDens = density;
			maxIdx = i;
		}
	}
	if (maxIdx < 0)
	{
		// Something is wrong...
		return location;
	}

	return vehicles[maxIdx].getLocation();
}

void Shark::reSpawn(std::vector<Fish>& vehicles)
{
	location = locateDensestFishPopulation(vehicles);
	float minSize = 6;
	float maxSize = 10;
	hunger = 0;
	isHunting = false;
	prayID = -1;
	timeAtLastMeal = GetTimeStamp();

	size = ofRandom(minSize, maxSize);
	setSizeAndSpeed(size);
}

void Shark::applyBehaviours(std::vector<Fish>& vehicles)
{
	updateBeachDetection();

	setSizeAndSpeed(size);
	if (!isHunting)
	{
		int t = GetTimeStamp();
		hunger = (t - timeAtLastMeal) / 5;
		if (hunger > 5)
		{
			if (ofRandom(100) < 10)
			{
				locatePray(vehicles);
			}
		}
	}
	
	if (isHunting)
	{
		huntF = huntEffect(vehicles);
	}

	if (hunger > size * 3)
	{
		// Die of hunger
		reSpawn(vehicles);

	}


	//alignF = 0.5 * alignEffect(vehicles);
	//cohesionF = 0.2 * cohesionEffect(vehicles);
	//separateF = separateEffect(vehicles);
	seekF = ofVec2f(0);
	//if (seekMother)
	//	seekF = seekMotherEffect();
	bordersF = bordersEffect();
	slopesF = slopesEffect();
	wanderF = wanderEffect();

	//    separateF*=1;//2;
	seekF *= 1;
	bordersF *= 2;
	slopesF *= 2;//2;
	wanderF *= 0.8;
	huntF *= 1.0;

	if (beach) {
		applyVelocityChange(slopesF);
	}
	if (border) {
		applyVelocityChange(bordersF);
	}
	//applyVelocityChange(separateF);
	//applyVelocityChange(alignF);
	//applyVelocityChange(cohesionF);
	applyVelocityChange(huntF);

	if (seekF.lengthSquared() == 0) {
		applyVelocityChange(wanderF);
	}
	else {
		applyVelocityChange(seekF);
	}
}

void Shark::draw()
{
	ofPushMatrix();
	ofTranslate(projectorCoord);
	if (DrawFlipped)
		ofRotate(180 + angle);
	else
		ofRotate(angle);

	// Compute tail angle
	float nv = 0.5;//velocity.lengthSquared()/10; // Tail movement amplitude
	float fact = 50 + 250 * velocity.length() / topSpeed;
	float tailangle = nv / 25 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50) - 25);

	// Color of the fish
	nv = 255;
	fact = 50;
//	float hsb = nv / 50 * (abs(((int)(ofGetElapsedTimef()*fact) % 100) - 50));

	// Fish scale
	float sc = size;
	float tailSize = 1 * sc;
	float fishLength = 2 * sc;
	float fishHead = tailSize;

	ofPolyline fish;
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8)));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8)));
	fish.curveTo(ofPoint(-fishLength, 0));
	fish.curveTo(ofPoint(0, -fishHead));
	fish.curveTo(ofPoint(fishHead, 0));
	fish.curveTo(ofPoint(0, fishHead));
	fish.curveTo(ofPoint(-fishLength, 0));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8)));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8)));
	fish.close();
	ofSetLineWidth(2.0);


	ofColor c = ofColor(255);
	ofColor StomachCol = ofColor(255);
	if (isHunting)
	{
//		c.setHsb((int)hsb, 255, 255); // rainbow
		// Red when hunting
		StomachCol = ofColor(255, 0, 0);
	}
	else if (hunger > size)
	{
		// Black grey when hungry
		StomachCol = ofColor(0, 0, 0);
	}
	else // not hunting or hungry - chill color
	{
		StomachCol = ofColor(255, 255, 255);
	}
	ofSetColor(c);
	ofFill();
	fish.draw();
	//if (isHunting)
	//{
	//	c.setHsb(255 - (int)hsb, 255, 255); // rainbow
	//	ofSetColor(c);
	//}
	ofSetColor(StomachCol);
	ofDrawCircle(0, 0, sc*0.5);
	ofNoFill();
	ofPopMatrix();
}

ofPoint Shark::wanderEffect()
{
	ofPoint velocityChange, desired;

	wandertheta += ofRandom(-change, change);     // Randomly change wander theta

	ofPoint front = velocity;
	front.normalize();
	front *= wanderD;
	ofPoint circleloc = location + front;

	float h = ofRadToDeg(atan2(front.y, front.x)); // Signed angle

	ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta + h), wanderR*sin(wandertheta + h));
	ofPoint target = circleloc + circleOffSet;

	desired = target - location;
	desired.normalize();
	desired *= topSpeed;

	velocityChange = desired - velocity;
	velocityChange.limit(maxVelocityChange);

	return velocityChange;
}

DangerousBOID::DangerousBOID(ofPoint loc, ofPoint vel, double sz)
{
	location = loc;
	velocity = vel;
	size = sz;
}
