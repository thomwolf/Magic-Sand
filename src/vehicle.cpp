#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, ofRectangle sborders){
    acceleration.set(0, 0);
    velocity.set(0.0, 0.0);
    location.set(x,y);
    
    borders = sborders;
    minborderDist = 12;
    internalBorders = sborders;
    internalBorders.scaleFromCenter((sborders.width-minborderDist)/sborders.width, (sborders.height-minborderDist)/sborders.height);
    
    r = 12;
    desiredseparation = 24;
    maxForce = 0.1;
    topSpeed =3;
}

//--------------------------------------------------------------
ofPoint vehicle::bordersForce(){
    ofPoint desired;
    
    // Predict location 5 (arbitrary choice) frames ahead
    ofPoint predict(velocity);
    predict *= 10;
    ofPoint futureLocation(location);
    futureLocation += predict;
    
    ofPoint leave(location);
    if (!internalBorders.inside(futureLocation)){
        if (futureLocation.x < internalBorders.getLeft())
            leave.x = borders.getRight();
        if (futureLocation.y < internalBorders.getTop())
            leave.y = borders.getBottom();
        if (futureLocation.x > internalBorders.getRight())
            leave.x = borders.getLeft();
        if (futureLocation.y > internalBorders.getBottom())
            leave.y = borders.getTop();
    }
    
    leave -= location;
    leave.normalize();
    leave *= velocity.length();
//    leave += velocity;
    
    desired.set(leave);
    desired.normalize();
    desired *= topSpeed;
    
    ofPoint steer(desired);
    if (desired.length() != 0) {
        steer -= velocity;
        //            PVector steer = PVector.sub(desired, velocity);
        steer.limit(maxForce);
    }
    return steer;
}

//--------------------------------------------------------------
ofPoint vehicle::slopesForce(ofVec2f* gradient){
    ofPoint desired;
    
    return ofPoint(1, 0);
}

//--------------------------------------------------------------
ofPoint vehicle::seekForce(const ofPoint & target){
    ofPoint desired;
    desired = target - location;
    
    desired.normalize();
    desired*=topSpeed;
    
    ofPoint steer;
    steer = desired - velocity;
    steer.limit(maxForce);

    return steer;
}

//--------------------------------------------------------------
ofPoint vehicle::separateForce(vector<vehicle> vehicles){
//    float desiredseparation = r*2;
    ofPoint sum;
    int count = 0;
    
    vector<vehicle>::iterator other;
    for (other = vehicles.begin(); other < vehicles.end(); other++){
        float d = (location - other->getLocation()).length();
        if((d>0) && (d < desiredseparation)){
         
            ofPoint diff = location - other->getLocation();
            diff.normalize();
            diff /= d;
            sum+= diff;
            count ++;
        }
    }
    if(count > 0){
        sum /= count;
        sum.normalize();
        sum*=topSpeed;
        
        sum -= velocity;
        sum.limit(maxForce);
    }
    return sum;
}

//--------------------------------------------------------------
void vehicle::applyBehaviours(vector<vehicle> vehicles, ofVec2f* gradient, ofPoint target){

     separateF = separateForce(vehicles);
     seekF = seekForce(target);
     bordersF = bordersForce();
     slopesF = slopesForce(gradient);
    
    separateF*=1;//2;
    seekF *= 1;
    bordersF *=1;
    slopesF *= 1;//2;
    
//    applyForce(slopesF);
//    applyForce(bordersF);
//    applyForce(separateF);
    applyForce(seekF);
    currentForce = separateF+seekF+bordersF+slopesF;
}

//--------------------------------------------------------------
std::vector<ofVec2f> vehicle::getForces(void)
{
    std::vector<ofVec2f> Forces;
    Forces.push_back( separateF);
    Forces.push_back( seekF);
    Forces.push_back( bordersF);
    Forces.push_back( slopesF);
    return Forces;
}

//--------------------------------------------------------------
void vehicle::applyForce(const ofPoint & force){
    ofPoint f(force);
    acceleration += f;
}

//--------------------------------------------------------------
void vehicle::update(){
    velocity += acceleration;
    location += velocity;
    velocity.limit(topSpeed);
    acceleration *= 0;
}
