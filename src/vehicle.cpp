#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, ofRectangle sborders)/*, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution)*/{
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    wandertheta = 0;
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
    topSpeed =3;
}

//--------------------------------------------------------------
void vehicle::updateBeachDetection(bool sbeach, float sbeachDist, ofVec2f sbeachSlope){
     beach = sbeach;
     beachDist = sbeachDist;
     beachSlope = sbeachSlope;
}

////--------------------------------------------------------------
//void vehicle::updateBasePlaneEq(ofVec4f sbasePlaneEq){
//    basePlaneEq = sbasePlaneEq;
//}

//--------------------------------------------------------------
ofPoint vehicle::bordersEffect(){
    ofPoint desired, futureLocation;
    
    // Predict location 10 (arbitrary choice) frames ahead
    futureLocation = location + velocity*10;
    
    ofPoint target = location;
    if (!internalBorders.inside(futureLocation)){ // Go to the opposite direction
        if (futureLocation.x < internalBorders.getLeft())
            target.x = borders.getRight();
        if (futureLocation.y < internalBorders.getTop())
            target.y = borders.getBottom();
        if (futureLocation.x > internalBorders.getRight())
            target.x = borders.getLeft();
        if (futureLocation.y > internalBorders.getBottom())
            target.y = borders.getTop();
    }
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    ofPoint velocityChange(0);
    if (desired.length() != 0) {
        velocityChange = desired - velocity;
        velocityChange.limit(maxVelocityChange);
    }
    return velocityChange;
}

//--------------------------------------------------------------
ofPoint vehicle::slopesEffect(){
    ofPoint desired, velocityChange;
    
    if(beach){
        desired = beachSlope;
        desired.normalize();
        desired *= topSpeed;
        desired /= beachDist; // The closest the beach is, the more we want to avoid it

        velocityChange = desired - velocity;
        velocityChange.limit(maxVelocityChange);
    } else {
        velocityChange = ofPoint(0);
    }
    return velocityChange;
}

//--------------------------------------------------------------
ofPoint vehicle::seekEffect(const ofPoint & target){
    ofPoint desired;
    desired = target - location;
    
    float d = desired.length();
    desired.normalize();

//If we are closer than 100 pixels...
    if (d < 100) {
//...set the magnitude according to how close we are.
      desired *= ofMap(d,0,100,0,topSpeed);
    } else {
//Otherwise, proceed at maximum speed.
      desired *= topSpeed;
    }
    
    ofPoint velocityChange;
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);

    return velocityChange;
}

//--------------------------------------------------------------
ofPoint vehicle::separateEffect(vector<vehicle> vehicles){
//    float desiredseparation = r*2;
    ofPoint velocityChange;
    int count = 0;
    ofPoint diff;
    vector<vehicle>::iterator other;
    for (other = vehicles.begin(); other < vehicles.end(); other++){
        float d = (location - other->getLocation()).length();
        if((d>0) && (d < desiredseparation)){
            diff = location - other->getLocation();
            diff.normalize();
            diff /= d;
            velocityChange+= diff;
            count ++;
        }
    }
    if(count > 0){
        velocityChange /= count;
        velocityChange.normalize();
        velocityChange*=topSpeed;
        
        velocityChange -= velocity;
        velocityChange.limit(maxVelocityChange);
    }
    return velocityChange;
}
//--------------------------------------------------------------
ofPoint vehicle::wanderEffect(){
    
    ofPoint velocityChange, desired;
    
    float wanderR = 25;         // Radius for our "wander circle"
    float wanderD = 80;         // Distance for our "wander circle"
    float change = 0.3;
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
void vehicle::applyBehaviours(vector<vehicle> vehicles, ofPoint target){

    separateF = separateEffect(vehicles);
    seekF = seekEffect(target);
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
    
    separateF*=0;//2;
    seekF *= 0;
    bordersF *=2;
    slopesF *= 2;//2;
    wanderF *= 0.8;
    
    applyVelocityChange(slopesF);
    applyVelocityChange(bordersF);
    applyVelocityChange(separateF);
    applyVelocityChange(seekF);
    applyVelocityChange(wanderF);
//    currentForce = separateF+seekF+bordersF+slopesF;
}

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
    velocity += globalVelocityChange;
    velocity.limit(topSpeed);
    location += velocity;
    globalVelocityChange *= 0;
    
    float desiredAngle = ofRadToDeg(atan2(velocity.y,velocity.x));
    float angleChange = (desiredAngle - angle)*velocity.length()/topSpeed;
    angleChange = max(min(angleChange, maxRotation), -maxRotation);
    angle += angleChange;

}
