/***********************************************************************
BoidGameController.h - Controller for a Sandbox animal game
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/


#ifndef _BoidGameController_h_
#define _BoidGameController_h_

#include "vehicle.h"
#include "../KinectProjector/KinectProjector.h"

//! Controller for the BOID game
/** The seminal paper on BOIDS can be found here: http://www.red3d.com/cwr/papers/1987/boids.html
    A good tutorial is here: https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732 */
class CBoidGameController
{
	public:
		//! Default constructor
		CBoidGameController();

		//! Destructor
		virtual ~CBoidGameController();

		void setup(std::shared_ptr<KinectProjector> const& k);
		void update();

		void drawProjectorWindow();

		void drawMainWindow(float x, float y, float width, float height);

		// 0: absolute beginner, 1: beginner, 2: medium, 3: expert
		bool StartGame(int difficulty);

		bool StartSeekMotherGame();

		void setProjectorRes(ofVec2f& PR);

		void setKinectRes(ofVec2f& KR);

		void setKinectROI(ofRectangle &KROI);

		// Should debug files be dumped
		void setDebug(bool flag);

		void setupGui();

		bool isIdle();

	private:
		
		std::shared_ptr<KinectProjector> kinectProjector;

		void onButtonEvent(ofxDatGuiButtonEvent e);
		void onToggleEvent(ofxDatGuiToggleEvent e);
		void onSliderEvent(ofxDatGuiSliderEvent e);
		void UpdateGUI();

		void updateBOIDS();

		void PlayAndShowCountDown(int resultTime);

		void DrawScoresOnFBO();

		void DrawFinalScoresOnFBO();
	
		void ComputeScores();

		std::string DataBaseDir;

		// 0: absolute beginner, 1: beginner, 2: medium, 3: expert
		int GameDifficulty;

		// Projector and kinect variables
		ofVec2f projRes;
		ofVec2f kinectRes;
		ofRectangle kinectROI;

		// Drawable area
		ofRectangle projROI;

		ofTrueTypeFont scoreFont;
		ofTrueTypeFont scoreFontSmall;
		ofTrueTypeFont nameFont;
		std::string ScoreText;
		std::string HiScoreText;

		// Should the boids be drawn flipped
		// Depending on the direction of the Kinect
		bool doFlippedDrawing;

		float LastTimeEvent;

		enum eGameState
		{
			GAME_STATE_IDLE,
			GAME_STATE_SHOWSPLASHSCREEN,
			GAME_STATE_PLAYANDSHOWCOUNTDOWN,
			GAME_STATE_SHOWINTERMIDEATERESULT,
			GAME_STATE_SHOWFINALRESULT
		};

		bool InitiateGameSequence();
		bool CreateSplashScreen();
		
		ofImage splashScreen;

		std::vector<eGameState> GameSequence;
		std::vector<int> GameSequenceTimings;
		int CurrentGameSequence;

		void SetupGameSequence();

		bool debugOn;
		std::string debugBaseDir;

		void addNewFish();
		void addNewShark();
		void addNewRabbit();
		bool addMotherFish();
		bool addMotherRabbit();
		bool setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location);

		void drawVehicles();
		void drawMotherFish();
		void drawMotherRabbit();

		// FBos
		ofFbo fboVehicles;

		// Animals
		vector<Fish> fish;
		vector<Rabbit> rabbits;
		vector<Shark> sharks;
		vector<DangerousBOID> dangerBOIDS;

		// Fish and Rabbits mothers
		ofPoint motherFish;
		ofPoint motherRabbit;
		bool showMotherFish;
		bool showMotherRabbit;
		float motherPlatformSize;

		// Scores
		double Player1Score;
		double Player2Score;
		double Player1Food;
		double Player2Food;
		double Player1Skins;
		double Player2Skins;

		// GUI
		ofxDatGui* gui;
};

#endif
