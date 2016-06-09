#include "vehicle.h"

//--------------------------------------------------------------
void vehicle::setup(int x, int y, ofRectangle sborders, ofMatrix4x4 skinectWorldMatrix, ofVec4f sbasePlaneEq, int skinectResX, int sgradFieldcols, int sgradFieldrows, double sgradFieldresolution){
    acceleration.set(0, 0);
    velocity.set(0.0, 0.0);
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
    maxForce = 0.1;
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
//    leave.normalize();
//    leave *= velocity.length();
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
ofPoint vehicle::slopesForce(){
    ofPoint desired;
    
    // Predict location 5 (arbitrary choice) frames ahead
    ofPoint predict(velocity);
    predict *= 10;
    ofPoint futureLocation(location);
    futureLocation += predict;
    
    // Get kinect depth image coord
    ofVec3f worldPoint = ofVec3f(futureLocation);
    worldPoint.z = FilteredDepthImage.getFloatPixelsRef().getData()[(int)futureLocation.x+kinectResX*(int)futureLocation.y];
    ofVec4f wc = ofVec4f(worldPoint);
    wc.w = 1;
    
    /* Transform the vertex from depth image space to world space: */
    //    ofVec3f vertexCcxx = kinectgrabber.kinect.getWorldCoordinateAt(vertexDic.x, vertexDic.y, vertexDic.z);
    ofVec4f vertexCc = kinectWorldMatrix*wc*wc.z;
    vertexCc.w = 1;
    
    /* Plug camera-space vertex into the base plane equation: */
    float elevation=-basePlaneEq.dot(vertexCc);///vertexCc.w;
    
    ofPoint grd(0);
    if (elevation > 0)
    {
        grd = -gradient[(int)(futureLocation.x/gradFieldresolution)+gradFieldcols*((int)(futureLocation.y/gradFieldresolution))];
    }
    grd.normalize();
    
    grd *= 10;
    ofPoint leave(location);
    leave += grd;
    
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
void vehicle::applyBehaviours(vector<vehicle> vehicles, ofPoint target){

     separateF = separateForce(vehicles);
     seekF = seekForce(target);
     bordersF = bordersForce();
     slopesF = slopesForce();
    
    separateF*=0;//2;
    seekF *= 1;
    bordersF *=1;
    slopesF *= 2;//2;
    
    applyForce(slopesF);
    applyForce(bordersF);
    applyForce(separateF);
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
