#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, int sscreenWidth, int sscreenHeight){
    acceleration.set(0, 0);
    velocity.set(0.0, 0.0);
    location.set(x,y);
    screenWidth = sscreenWidth;
    screenHeight = sscreenHeight;
    
    r = 12;
    border = 12;
    desiredseparation = 24;
    maxForce = 0.1;
    topSpeed =3;
}

//--------------------------------------------------------------
ofPoint vehicle::borders(){
    ofPoint desired;
    
    // Predict location 5 (arbitrary choice) frames ahead
    ofPoint predict(velocity);
    predict *= 10;
    ofPoint futureLocation(location);
    futureLocation += predict;
    
    ofPoint leave(location);
    if (futureLocation.x < border)
        leave.x = screenWidth;
    if (futureLocation.y < border)
        leave.y = screenHeight;
    if (futureLocation.x > screenWidth-border)
        leave.x = 0;
    if (futureLocation.y > screenHeight-border)
        leave.y = 0;
    
        leave -= location;
        leave.normalize();
        leave *= velocity.length();
        leave += velocity;
        
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
ofPoint vehicle::slopes(ofVec2f* gradient){
    ofPoint desired;
    
    // Predict location 5 (arbitrary choice) frames ahead
    ofPoint predict(velocity);
    predict *= 10;
    ofPoint futureLocation(location);
    futureLocation += predict;
    
    ofPoint leave(location);
    if (futureLocation.x < border)
        leave.x = screenWidth;
    if (futureLocation.y < border)
        leave.y = screenHeight;
    if (futureLocation.x > screenWidth-border)
        leave.x = 0;
    if (futureLocation.y > screenHeight-border)
        leave.y = 0;
    
    leave -= location;
    leave.normalize();
    leave *= velocity.length();
    leave += velocity;
    
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
ofPoint vehicle::seek(const ofPoint & target){
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
ofPoint vehicle::separate(vector<vehicle> vehicles){
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
void vehicle::applyBehaviours(vector<vehicle> vehicles, ofVec2f* gradient){

    ofPoint mouse(ofGetMouseX(), ofGetMouseY());
    
    ofPoint separateForce = separate(vehicles);
    ofPoint seekForce = seek(mouse);
    ofPoint border = borders();
    ofPoint slope = slopes(gradient);
    
    separateForce*=2;
    seekForce *= 1;
    border *=3;
    slope *= 2;
    
    applyForce(slope);
    applyForce(border);
    applyForce(separateForce);
    applyForce(seekForce);
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

//--------------------------------------------------------------
void vehicle::draw(){
    //float angle = ofRadToDeg(atan2(velocity.y,velocity.x)) + 90;
    
    ofPushMatrix();
  
    ofSetColor(255);
    ofTranslate(location.x, location.y);
    //ofRotate(angle);
    ofDrawCircle(0, 0, r/2);
    ofPopMatrix();
}
