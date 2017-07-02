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
    resetBurnedArea();

}

void Model::addNewFire(){
    ofVec2f location;
    setRandomVehicleLocation(kinectROI, false, location);
    auto f = Fire(kinectProjector, location, kinectROI);
    f.setup();
    fires.push_back(f);
}

void Model::addNewFire(ofVec2f fireSpawnPos) {
    addNewFire(fireSpawnPos, 0);
}

void Model::addNewFire(ofVec2f fireSpawnPos, float angle){
    if (kinectProjector->elevationAtKinectCoord(fireSpawnPos.x, fireSpawnPos.y) < 0){
        return;
    }
    auto f = Fire(kinectProjector, fireSpawnPos, kinectROI, angle);
    f.setup();
    fires.push_back(f);

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

// SETTERS and GETTERS Fire Parameters:
void Model::setWindspeed(float uiwindspeed) {
	windspeed = uiwindspeed;
}

void Model::setTemp(float uiTemp) {
	temperature = uiTemp;
}

void Model::setWinddirection(float uiWinddirection) {
	winddirection = uiWinddirection;
}

void Model::update(){
    // kinectROI updated
    if (kinectProjector->getKinectROI() != kinectROI){
        kinectROI = kinectProjector->getKinectROI();
        resetBurnedArea();
    }
    
	
    
    //spread fires
    int size = fires.size();
    for (int i = 0; i < size ; i++){
        ofPoint location = fires[i].getLocation();
        if (burnedArea[floor(location.x)][floor(location.y)]){
            fires[i].kill();
        } else {
            burnedArea[floor(location.x)][floor(location.y)] = true;
        }
        
        int rand = std::rand() % 100;
		int spreadFactor = 10;
		if (temperature > 30) {
			spreadFactor = 20;
		}
		else if (temperature < 10) {
			spreadFactor = 5;
		}
        if (fires[i].isAlive() && rand < spreadFactor){
            int angle = fires[i].getAngle();
            addNewFire(location, (angle + 90)%360);
            addNewFire(location, (angle + 270)%360);
        }
    }
	deleteDeadFires();
    for (auto & f : fires){
        if(f.isAlive()){
            f.applyBehaviours(temperature,windspeed,winddirection);
            f.update();
        }
    }
}

void Model::draw(){
    for (auto & f : fires){
        f.draw();
    }
}

void Model::clear(){
    fires.clear();
    resetBurnedArea();
}

void Model::deleteDeadFires() {
	vector<int> deadFires;
	
	for (int i = 0; i < fires.size(); i++) {
		if (!fires[i].isAlive()) {
			deadFires.push_back(i);
		}
	}
	for (int i = 0; i < deadFires.size(); i++) {
		fires.erase(fires.begin() + (deadFires[i] - i));
	}
}
void Model::resetBurnedArea(){
    burnedArea.clear();
    for(int x = 0; x <= kinectROI.getRight(); x++ ){
        vector<bool> row;
        for (int y = 0; y <= kinectROI.getBottom(); y++ ){
            row.push_back(false);
        }
        burnedArea.push_back(row);
    }
}

