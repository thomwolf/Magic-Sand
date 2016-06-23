#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, ofRectangle sborders)/*, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution)*/{
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    wandertheta = 0;
    currentStraightPathLength = 0;
    mother = false;
    location.set(x,y);
    
    //    kinectWorldMatrix = skinectWorldMatrix;
    //    basePlaneEq = sbasePlaneEq;
    //    kinectResX = skinectResX;
    //    gradFieldcols = sgradFieldcols;
    //    gradFieldrows = sgradFieldrows;
    //    gradFieldresolution = sgradFieldresolution;
    
    borders = sborders;
    minborderDist = 12;
    internalBorders = sborders;
    internalBorders.scaleFromCenter((sborders.width-minborderDist)/sborders.width, (sborders.height-minborderDist)/sborders.height);
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 30;
    topSpeed =2;
    velocityIncreaseStep = 0.5;
    maxStraightPath = 50;
}

//--------------------------------------------------------------
void vehicle::updateBeachDetection(bool sbeach, float sbeachDist, ofVec2f sbeachSlope){
    beach = sbeach;
    beachDist = sbeachDist;
    beachSlope = sbeachSlope;
}

//--------------------------------------------------------------
ofPoint vehicle::bordersEffect(){
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
//--------------------------------------------------------------
ofPoint vehicle::wanderEffect(){
    
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

//--------------------------------------------------------------
ofPoint vehicle::slopesEffect(){
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

//--------------------------------------------------------------
ofPoint vehicle::seekEffect(const ofPoint & target){
//    ofPoint desired;
//    desired = target - location;
//    
//    float d = desired.length();
//    desired.normalize();
//    
//    //If we are closer than XX pixels slow down
//    if (d < 10) {
//        desired *= ofMap(d,0,100,0,topSpeed);
//        mother = true;
//    } else {
//        //Otherwise, proceed at maximum speed.
//        desired *= topSpeed;
//    }
//    
//    ofPoint velocityChange;
//    velocityChange = desired - velocity;
//    velocityChange.limit(maxVelocityChange);
//    
//    //If we are further than XX pixels we don't see the mother
//    if (d > 50) {
//        velocityChange = ofPoint(0);
//    }
//    
//    return velocityChange;
    return ofPoint(0);
}

//--------------------------------------------------------------
//ofPoint vehicle::separateEffect(vector<vehicle> vehicles){
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

//--------------------------------------------------------------
std::vector<ofVec2f> vehicle::getForces(void)
{
    std::vector<ofVec2f> Forces;
    Forces.push_back( separateF);
    Forces.push_back( seekF);
    Forces.push_back( bordersF);
    Forces.push_back( slopesF);
    Forces.push_back( wanderF);
    return Forces;
}

//--------------------------------------------------------------
void vehicle::applyVelocityChange(const ofPoint & velocityChange){
    globalVelocityChange += velocityChange;
}

//--------------------------------------------------------------
void vehicle::update(){
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


ofPoint Fish::wanderEffect(){
    
    ofPoint velocityChange, desired;
    
    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofPoint front = velocity;
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
    float h = ofRadToDeg(atan2(front.y,front.x));
	//float h = front.angle(ofVec2f(1,0)); // !! angle gives unsigned angles !
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
    
    return velocityChange;
}

//--------------------------------------------------------------
void Fish::setup(int x, int y, ofRectangle sborders)/*, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution)*/{
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    wandertheta = 0;
    beach = false;
    border = false;
    mother = false;
    currentStraightPathLength = 0;
    location.set(x,y);
    
    //    kinectWorldMatrix = skinectWorldMatrix;
    //    basePlaneEq = sbasePlaneEq;
    //    kinectResX = skinectResX;
    //    gradFieldcols = sgradFieldcols;
    //    gradFieldrows = sgradFieldrows;
    //    gradFieldresolution = sgradFieldresolution;
    
    borders = sborders;
    minborderDist = 50;
    internalBorders = sborders;
    internalBorders.scaleFromCenter((sborders.width-minborderDist)/sborders.width, (sborders.height-minborderDist)/sborders.height);
    
    wanderR = 10;         // Radius for our "wander circle"
    wanderD = 80;         // Distance for our "wander circle"
    change = 0.3;
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 30;
    topSpeed =2;
    velocityIncreaseStep = 0;
    maxStraightPath = 50;
    minVelocity = 0;
}
//--------------------------------------------------------------
void Fish::applyBehaviours(/*vector<vehicle> vehicles, */ofPoint target){
    
    //    separateF = separateEffect(vehicles);
    seekF = seekEffect(target);
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

//==============================================================
// Derived class Rabbit
//==============================================================

//--------------------------------------------------------------
void Rabbit::setup(int x, int y, ofRectangle sborders)/*, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution)*/{
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    wandertheta = 0;
    currentStraightPathLength = 0;
    beach = false;
    border = false;
    setWait = false;
    mother = false;
    location.set(x,y);
    
    //    kinectWorldMatrix = skinectWorldMatrix;
    //    basePlaneEq = sbasePlaneEq;
    //    kinectResX = skinectResX;
    //    gradFieldcols = sgradFieldcols;
    //    gradFieldrows = sgradFieldrows;
    //    gradFieldresolution = sgradFieldresolution;
    
    borders = sborders;
    minborderDist = 50;
    internalBorders = sborders;
    internalBorders.scaleFromCenter((sborders.width-minborderDist)/sborders.width, (sborders.height-minborderDist)/sborders.height);
    
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
}
//--------------------------------------------------------------
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
//--------------------------------------------------------------
void Rabbit::applyBehaviours(ofPoint target){
    //    separateF = separateEffect(vehicles);
    seekF = seekEffect(target);
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
                setWait=true;
                waitCounter = 0;
                waitTime = ofRandom(minWaitingTime, maxWaitingTime);
                if (beach)
                    waitTime = 0;
            }
        }
    }
}

