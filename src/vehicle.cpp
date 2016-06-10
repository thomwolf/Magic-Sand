#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, ofRectangle sborders, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution){
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    angle = 0;
    location.set(x,y);
    
    kinectWorldMatrix = skinectWorldMatrix;
    basePlaneEq = sbasePlaneEq;
    kinectResX = skinectResX;
    gradFieldcols = sgradFieldcols;
    gradFieldrows = sgradFieldrows;
    gradFieldresolution = sgradFieldresolution;
    
    borders = sborders;
    minborderDist = 12;
    internalBorders = sborders;
    internalBorders.scaleFromCenter((sborders.width-minborderDist)/sborders.width, (sborders.height-minborderDist)/sborders.height);
    
    r = 12;
    desiredseparation = 24;
    maxVelocityChange = 1;
    maxRotation = 10;
    topSpeed =3;
}

//--------------------------------------------------------------
void vehicle::updateFilteredDepthImageAndGradient(ofxCvFloatImage sFilteredDepthImage, ofVec2f* sgradient){
    FilteredDepthImage = sFilteredDepthImage;
    gradient = sgradient;
}

//--------------------------------------------------------------
void vehicle::updateBasePlaneEq(ofVec4f sbasePlaneEq){
    basePlaneEq = sbasePlaneEq;
}

//--------------------------------------------------------------
ofPoint vehicle::bordersEffect(){
    ofPoint desired, futureLocation, predict;
    
    // Predict location 10 (arbitrary choice) frames ahead
    predict = velocity*10;
    futureLocation = location + predict;
    
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
    ofPoint futureLocation = location;
    ofPoint desired = ofPoint(0);
    
    // Find first surface out of the water in the 10 future frame steps from kinectdepth, conversion matrices and baseplane equation
    int i = 1;
    bool water = true;
    while (i < 10 && water){
        ofVec4f wc = ofVec3f(futureLocation);
        wc.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)futureLocation.x+kinectResX*(int)futureLocation.y];
        wc.w = 1;
        ofVec4f vertexCc = kinectWorldMatrix*wc*wc.z;
        vertexCc.w = 1;
        float elevation=-basePlaneEq.dot(vertexCc);///vertexCc.w;
        
        if (elevation > 0) // Our fish want to stay in the water, take the gradient field direction at the futurelocation
        {
            desired = -gradient[(int)(futureLocation.x/gradFieldresolution)+gradFieldcols*((int)(futureLocation.y/gradFieldresolution))];
            desired.normalize();
            desired *= topSpeed;
            desired /= i; // The closest the beach is, the more we want to avoid it
            water = false;
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
    
    ofPoint velocityChange(0);
    if (desired.length() != 0) {
        velocityChange = desired - velocity;
        velocityChange.limit(maxVelocityChange);
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
void vehicle::applyBehaviours(vector<vehicle> vehicles, ofPoint target){

     separateF = separateEffect(vehicles);
     seekF = seekEffect(target);
     bordersF = bordersEffect();
     slopesF = slopesEffect();
    
    separateF*=1;//2;
    seekF *= 1;
    bordersF *=1;
    slopesF *= 2;//2;
    
    applyVelocityChange(slopesF);
    applyVelocityChange(bordersF);
    applyVelocityChange(separateF);
    applyVelocityChange(seekF);
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
    
    float desiredAngle = velocity.angle(ofVec2f(1,0)); // angle of the velocity with left vector
    float angleChange = (desiredAngle - angle)*velocity.lengthSquared();
    angleChange = max(min(angleChange, maxRotation), -maxRotation);
    angle += angleChange;

}
