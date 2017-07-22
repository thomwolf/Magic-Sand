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

	bool isRunning();

    void setWindSpeed(float v);
    void setWindDirection(float d);

    void addNewFire();
    void addNewFire(ofVec2f fireSpawnPos);
    void addNewFire(ofVec2f fireSpawnPos, float angle);

    void calculateRiskZones();
	void drawRiskZones();

    void update();
    void draw();
    void clear();

private:
    std::shared_ptr<KinectProjector> kinectProjector;
    ofRectangle kinectROI;
    
    vector<Fire> fires;
    vector<Fire> embers;
    vector< vector<bool> > riskZones;
    vector< vector<bool> > burnedArea;
    
    float windSpeed;
    float windDirection;

    bool setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location);

	void resetBurnedArea();
    void drawEmbers();
};
