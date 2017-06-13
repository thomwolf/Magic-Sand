//
//  Model.cpp
//  Magic-Sand
//
//  Created by Jan on 06/06/17.
//
//

#include "Model.h"

Model::Model(std::shared_ptr<KinectProjector> const& k){
    kinectProjector = k;
    
    
    // Retrieve variables
    kinectROI = kinectProjector->getKinectROI();

}

void Model::addNewFire(){
    ofVec2f location;
    setRandomVehicleLocation(kinectROI, false, location);
    auto f = Fire(kinectProjector, location, kinectROI);
    f.setup();
    fires.push_back(f);
}

void Model::addNewFire(ofVec2f fireSpawnPos) {
    ofVec2f location;
    setFixedVehicleLocation(fireSpawnPos, false, location);
    auto f = Fire(kinectProjector, location, kinectROI);
    f.setup();
    fires.push_back(f);
}

//Fixed Position for Rabbits : Simon
bool Model::setFixedVehicleLocation(ofVec2f pos, bool liveInWater, ofVec2f & location){
    bool okwater = false;
    int countFixed = 0;
    int maxCount = 100;
    while (!okwater && countFixed < maxCount) {
        countFixed++;
        bool insideWater = kinectProjector->elevationAtKinectCoord(pos.x, pos.y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater)) {
            location = pos;
            okwater = true;
        }
    }
    return okwater;
}


bool Model::setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location){
    bool okwater = false;
    int count = 0;
    int maxCount = 100;
    while (!okwater && count < maxCount) {
        count++;
        float x = ofRandom(area.getLeft(),area.getRight());
        float y = ofRandom(area.getTop(),area.getBottom());
        bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater)){
            location = ofVec2f(x, y);
            okwater = true;
        }
    }
    return okwater;
}

void Model::update(){
    if (kinectProjector->isROIUpdated())
        kinectROI = kinectProjector->getKinectROI();
    
    for (auto & f : fires){
        f.applyBehaviours();
        f.update();
    }
}

void Model::draw(){
    for (auto & f : fires){
        f.draw();
    }
}

void Model::clear(){
    fires.clear();
}

