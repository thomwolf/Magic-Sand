//
//  Model.h
//  Magic-Sand
//
//  Created by Jan on 06/06/17.
//
//

#pragma once

#include "ofMain.h"
#include "KinectProjector/KinectProjector.h"
#include "vehicle.h"


class Model{
public:
    Model(std::shared_ptr<KinectProjector> const& k);
    
    void addNewFire();
    void addNewFire(ofVec2f fireSpawnPos);
    
    bool setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location);
    
    void update();
    
    void draw();
    
    void clear();
    
private:
    std::shared_ptr<KinectProjector> kinectProjector;
    
    ofVec2f kinectRes;
    ofRectangle kinectROI;
    
    // Fire
    vector<Fire> fires;
};
