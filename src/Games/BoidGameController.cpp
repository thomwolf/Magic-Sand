/***********************************************************************
BoidGameController.cpp - Controller for a Sandbox animal game
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


#include "BoidGameController.h"

#include <string>
//#include <direct.h>

CBoidGameController::CBoidGameController()
{
	DataBaseDir = "boidGame/";
	setDebug(true);

	int xmin = 0;
	int ymin = 0;
	int xmax = 640;
	int ymax = 480;
		
	std::string SplashScreenFname = DataBaseDir + "art/BoidGameSplashScreen.png";
	if (!splashScreen.loadImage(SplashScreenFname))
	{
		ofLogVerbose("CBoidGameController()") << "could not read splash screen ";
	}
	GameDifficulty = 2;

	LastTimeEvent = ofGetElapsedTimef();
	SetupGameSequence();
	doFlippedDrawing = false;
}

CBoidGameController::~CBoidGameController()
{
}

void CBoidGameController::setup(std::shared_ptr<KinectProjector> const& k)
{
	kinectProjector = k;

	ofTrueTypeFont::setGlobalDpi(72);
	if (!scoreFont.loadFont("verdana.ttf", 64))
		std::cout << "Could not read font verdana.ttf" << std::endl;
	if (!scoreFontSmall.loadFont("verdana.ttf", 32))
		std::cout << "Could not read font verdana.ttf" << std::endl;
	if (!nameFont.loadFont("verdana.ttf", 128))
		std::cout << "Could not read font verdana.ttf" << std::endl;

	// Vehicles
	showMotherFish = false;
	showMotherRabbit = false;
	motherPlatformSize = 30;
	doFlippedDrawing = kinectProjector->getProjectionFlipped();

	setupGui();
}



void CBoidGameController::updateBOIDS()
{
	// Set static varible that indicate if all BOIDS should be drawn flipped
	Vehicle::setDrawFlipped(doFlippedDrawing);

	if (kinectProjector->isImageStabilized()) {
		for (auto & f : fish) {
			f.applyBehaviours(showMotherFish, fish, dangerBOIDS);
			f.update();
		}
		for (auto & r : rabbits) {
			r.applyBehaviours(showMotherRabbit, rabbits);
			r.update();
		}
		dangerBOIDS.clear();
		for (auto & s : sharks) {
			s.applyBehaviours(fish);
			s.update();
			dangerBOIDS.push_back(DangerousBOID(s.getLocation(), s.getVelocity(), s.getSize() * 4));
		}
		drawVehicles();
	}
}



void CBoidGameController::update()
{
	float resultTime = ofGetElapsedTimef();



	eGameState sequence = GameSequence[CurrentGameSequence];
	//if (sequence == GAME_STATE_SHOWMAPNAME)
	//{
	//	// Do nothing in update
	//}
	if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		fboVehicles.begin();
		ofClear(255, 255, 255, 0);

		updateBOIDS();
		ComputeScores();
		DrawScoresOnFBO();
		PlayAndShowCountDown(resultTime);

		fboVehicles.end();
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT)
	{
		// Do nothing in update
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		// Do nothing in update
	}
	else if (sequence == GAME_STATE_IDLE)
	{
		fboVehicles.begin();
		ofClear(255, 255, 255, 0);

		updateBOIDS();

		fboVehicles.end();
	}

	int DeltaTime = GameSequenceTimings[CurrentGameSequence];
	if (DeltaTime > 0)  // -1 is infinite
	{
		if (resultTime - LastTimeEvent > DeltaTime)
		{
			CurrentGameSequence++;
			// Debug check
			if (CurrentGameSequence >= GameSequence.size())
				CurrentGameSequence = GameSequence.size() - 1;

			InitiateGameSequence();
		}
	}
	gui->update();
}

void CBoidGameController::ComputeScores()
{
	Player1Score = 0;
	Player2Score = 0;

	double xmid = kinectROI.x + kinectROI.width / 2;

	double FishMassP1 = 0;
	double FishMassP2 = 0;
	for (int i = 0; i < fish.size(); i++)
	{
		ofPoint loc = fish[i].getLocation();
		double size = fish[i].getSize();

		if (loc.x < xmid)
		{
			FishMassP2 += size;
		}
		else
		{
			FishMassP1 += size;
		}
	}
	Player1Food += FishMassP1 / 2000.0;
	Player2Food += FishMassP2 / 2000.0;

	double NrabP1 = 0;
	double NrabP2 = 0;

	for (int i = 0; i < rabbits.size(); i++)
	{
		ofPoint loc = rabbits[i].getLocation();

		if (loc.x < xmid)
		{
			NrabP2 += 1;
		}
		else
		{
			NrabP1 += 1;
		}
	}
	Player1Skins += NrabP1 / 200.0;
	Player2Skins += NrabP2 / 200.0;

	Player1Score = Player1Skins + Player1Food;
	Player2Score = Player2Skins + Player2Food;
}

void CBoidGameController::PlayAndShowCountDown(int resultTime)
{
	int DeltaTime = GameSequenceTimings[CurrentGameSequence];
	int timeleft = DeltaTime - (resultTime - LastTimeEvent);

	std::string timeString = ofToString(timeleft);

	double sW = scoreFont.stringWidth(timeString);
	double sH = scoreFont.stringHeight(timeString);

	double sx = projROI.x + (projROI.width / 2 - sW/2);
	double sy = projROI.y + 70;

	scoreFont.drawString(timeString, sx, sy);
}

void CBoidGameController::DrawScoresOnFBO()
{
	ofSetColor(250, 250, 210);

	//std::string P1ScoreStr = "Fish mass: " + ofToString((int)Player1Score) + "\n" +
	//	"Food: " + ofToString((int)Player1Food) + "\n" +
	//	"Skins: " + ofToString((int)Player1Skins) + "\n" +
	//	"Score: " + ofToString((int)(Player1Skins + Player1Food));

	std::string P1ScoreStr = 
		"Food: " + ofToString((int)Player1Food) + "\n" +
		"Skins: " + ofToString((int)Player1Skins) + "\n" +
		"Score: " + ofToString((int)(Player1Skins + Player1Food));

	double sW = scoreFontSmall.stringWidth(P1ScoreStr);
	double sH = scoreFontSmall.stringHeight(P1ScoreStr);

	double sx = projROI.x + (projROI.width / 4 - sW / 2);
	double sy = projROI.y + 70;

	scoreFontSmall.drawString(P1ScoreStr, sx, sy);

	//std::string P2ScoreStr = "P2: " + ofToString((int)Player2Score);
	std::string P2ScoreStr = 
		"Food: " + ofToString((int)Player2Food) + "\n" +
		"Skins: " + ofToString((int)Player2Skins) + "\n" +
		"Score: " + ofToString((int)(Player2Skins + Player2Food));

	//std::string P2ScoreStr = "Fish mass: " + ofToString((int)Player2Score) + "\n" +
	//	"Food: " + ofToString((int)Player2Food) + "\n" +
	//	"Skins: " + ofToString((int)Player2Skins) + "\n" +
	//	"Score: " + ofToString((int)(Player2Skins + Player2Food));


	sW = scoreFontSmall.stringWidth(P1ScoreStr);
	sH = scoreFontSmall.stringHeight(P1ScoreStr);

	sx = projROI.x + (3 * projROI.width / 4 - sW / 2);
	sy = projROI.y + 70;

	scoreFontSmall.drawString(P2ScoreStr, sx, sy);

	double xmid = kinectROI.x + kinectROI.width / 2;
	double ymid = kinectROI.y + kinectROI.height / 2;

	ofVec2f projMid = kinectProjector->kinectCoordToProjCoord(xmid, ymid);

	ofSetColor(0, 255, 255);
	ofSetLineWidth(3.0f);
	ofVec2f ps(projMid.x, projROI.getMinY());
	ofVec2f pe(projMid.x, projROI.getMaxY());

	ofDrawLine(ps, pe);
}


void CBoidGameController::DrawFinalScoresOnFBO()
{
	DrawScoresOnFBO();

	ofSetColor(255, 255, 255);
	Player1Score = Player1Skins + Player1Food;
	Player2Score = Player2Skins + Player2Food;

	std::string scorestr = "You won!";
	double sW = nameFont.stringWidth(scorestr);
	double sH = nameFont.stringHeight(scorestr);

	double sy = projROI.y + (projROI.height / 2 - sH / 2);
	double sx = 0;

	if (Player1Score > Player2Score)
	{
		sx = projROI.x + (projROI.width / 4 - sW / 2);
	}
	else
	{
		sx = projROI.x + (3 * projROI.width / 4 - sW / 2);

	}

	scoreFont.drawString(scorestr, sx, sy);
}

void CBoidGameController::drawMainWindow(float x, float y, float width, float height) 
{
	fboVehicles.draw(x, y, width, height);
	gui->draw();
}


void CBoidGameController::drawProjectorWindow()
{
	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence == GAME_STATE_SHOWSPLASHSCREEN)
	{
		fboVehicles.draw(0, 0);
	}
	else if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		fboVehicles.draw(0, 0);
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT)
	{
		fboVehicles.draw(0, 0);
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		fboVehicles.draw(0, 0);
	}
	else if (sequence == GAME_STATE_IDLE)
	{
		fboVehicles.draw(0, 0);
	}
}


bool  CBoidGameController::InitiateGameSequence()
{
	eGameState sequence = GameSequence[CurrentGameSequence];

//	if (sequence == GAME_STATE_SHOWMAPNAME)
//	{
////		CreateWelcomeScreen();
//		std::cout << "Created welcome screen" << std::endl;
//	}
	if (sequence == GAME_STATE_SHOWSPLASHSCREEN)
	{
		CreateSplashScreen();
		std::cout << "Created splash screen" << std::endl;
	}
	else if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		fboVehicles.begin();
		ofClear(255, 255, 255, 0);

		fboVehicles.end();
		fish.clear();
		rabbits.clear();
		sharks.clear();
		showMotherFish = false;
		showMotherRabbit = false;

		int nFish = 50;
		int nRabbit = 10;
		int nShark = 2;

		if (GameDifficulty == 0)
		{
			nFish = 5;
			nRabbit = 1;
			nShark = 0;
		}
		else if (GameDifficulty == 1)
		{
			nFish = 15;
			nRabbit = 3;
			nShark = 1;
		}
		else if (GameDifficulty == 2)
		{
			nFish = 30;
			nRabbit = 6;
			nShark = 2;
		}
		else if (GameDifficulty == 3)
		{
			nFish = 50;
			nRabbit = 10;
			nShark = 2;
		}


		for (int i = 0; i < nFish; i++)
			addNewFish();

		for (int i = 0; i < nRabbit; i++)
			addNewRabbit();

		for (int i = 0; i < nShark; i++)
			addNewShark();

		Player1Food = 0;
		Player2Food = 0;
		Player1Skins = 0;
		Player2Skins = 0;

		UpdateGUI();
//		std::cout << "Playing with countdown started" << std::endl;
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT)
	{
		//
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		fboVehicles.begin();
		ofClear(255, 255, 255, 0);

		DrawFinalScoresOnFBO();

		fboVehicles.end();
	}

	LastTimeEvent = ofGetElapsedTimef();
	return true;
}


bool CBoidGameController::CreateSplashScreen()
{
	double sW = splashScreen.getWidth();
	double sH = splashScreen.getHeight();
	if (sW == 0 || sH == 0)
	{
		ofLogVerbose("CreateSplashScreen") << "splash screen invalid";
		return false;
	}

	// pH = 800, pW = 1280,  sH = 600, sW = 800
	// scale = 800 / 600
	// Swn = 800 / 600 * 800
	double scale = projROI.getHeight() / sH;
	double sWn = scale * sW;
	double sHn = projROI.getHeight();

	if (sWn > projROI.getWidth())
	{
		scale = projROI.getWidth() / sW;
		sHn = scale * sH;
		sWn = projROI.getWidth();
	}

	double xs = projROI.x + (projROI.width / 2 - sWn / 2);
	double ys = projROI.y + (projROI.height / 2 - sHn / 2);

	fboVehicles.begin();
	ofClear(255, 255, 255, 0);
	ofBackground(255);

	splashScreen.draw(xs, ys, sWn, sHn);

	fboVehicles.end();
	return true;
}

bool CBoidGameController::StartGame(int difficulty)
{
	GameDifficulty = difficulty;
	if (GameSequence[CurrentGameSequence] != GAME_STATE_IDLE)
	{
		// Stop the game
		CurrentGameSequence = GameSequence.size() - 1; // Set to IDLE by default
		return false;
	}

	projROI = kinectProjector->getProjectorActiveROI();
	
	CurrentGameSequence = 0;
	InitiateGameSequence();

	return true;
}

bool CBoidGameController::StartSeekMotherGame()
{
	if (GameSequence[CurrentGameSequence] != GAME_STATE_IDLE)
	{
		// Stop the current game
		CurrentGameSequence = GameSequence.size() - 1; // Set to IDLE by default
	}
	fboVehicles.begin();
	ofClear(255, 255, 255, 0);

	fboVehicles.end();
	fish.clear();
	rabbits.clear();
	sharks.clear();
	addMotherFish();
	addMotherRabbit();

	for (int i = 0; i < 20; i++)
		addNewFish();

	for (int i = 0; i < 10; i++)
		addNewRabbit();

	UpdateGUI();
	return true;
}

void CBoidGameController::setProjectorRes(ofVec2f& PR)
{
	projRes = PR;

	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0, 0, 0, 255);
	fboVehicles.end();
}

void CBoidGameController::setKinectRes(ofVec2f& KR)
{
	kinectRes = KR;
}

void CBoidGameController::setKinectROI(ofRectangle &KROI)
{
	kinectROI = KROI;
	doFlippedDrawing = kinectProjector->getProjectionFlipped();
}

void CBoidGameController::setDebug(bool flag)
{
	debugOn = flag;

	if (flag)
	{
		std::string BaseDir = "data//BoidGame//DebugFiles//";
//		system("mkdir" + BaseDir.c_str());

		// By default the application looks in data
		debugBaseDir = "BoidGame//DebugFiles//";
	}
}

void CBoidGameController::SetupGameSequence()
{
	GameSequence.push_back(GAME_STATE_SHOWSPLASHSCREEN);
	GameSequenceTimings.push_back(5);

	GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	GameSequenceTimings.push_back(180);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(30);

	//GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	//GameSequenceTimings.push_back(5);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(60);

	//GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	//GameSequenceTimings.push_back(5);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(20);

	//GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	//GameSequenceTimings.push_back(5);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(30);

	GameSequence.push_back(GAME_STATE_SHOWFINALRESULT);
	GameSequenceTimings.push_back(5);

	GameSequence.push_back(GAME_STATE_IDLE);
	GameSequenceTimings.push_back(-1); // Indefinite

	CurrentGameSequence = GameSequence.size()-1; // Set to IDLE by default
}

void CBoidGameController::addNewFish() {
	ofVec2f location;
	// Do not add fish at borders
	double W = kinectROI.getWidth() * 0.60;
	double H = kinectROI.getHeight() * 0.60;
	double X = kinectROI.getLeft() + 0.20 * W;
	double Y = kinectROI.getTop() + 0.20 * H;
	ofRectangle fishROI(X, Y, W, H);

	setRandomVehicleLocation(fishROI, true, location);
	auto f = Fish(kinectProjector, location, kinectROI, motherFish);
	f.setup();
	fish.push_back(f);

	//if (fish.size() / 30 > sharks.size())
	//	addNewShark();
}

void CBoidGameController::addNewShark()
{
	ofVec2f location;
	// Do not add  at borders
	double W = kinectROI.getWidth() * 0.60;
	double H = kinectROI.getHeight() * 0.60;
	double X = kinectROI.getLeft() + 0.20 * W;
	double Y = kinectROI.getTop() + 0.20 * H;
	ofRectangle ROI(X, Y, W, H);

	setRandomVehicleLocation(ROI, true, location);
	auto s = Shark(kinectProjector, location, kinectROI, motherFish);
	s.setup();
	sharks.push_back(s);

}

void CBoidGameController::addNewRabbit() {
	ofVec2f location;
	double W = kinectROI.getWidth() * 0.60;
	double H = kinectROI.getHeight() * 0.60;
	double X = kinectROI.getLeft() + 0.20 * W;
	double Y = kinectROI.getTop() + 0.20 * H;
	ofRectangle rabbitROI(X, Y, W, H);
	setRandomVehicleLocation(rabbitROI, false, location);
	auto r = Rabbit(kinectProjector, location, kinectROI, motherRabbit);
	r.setup();
	rabbits.push_back(r);
}

bool CBoidGameController::addMotherFish() {
	int minborderDist = 40;
	ofRectangle internalBorders = kinectROI;
	internalBorders.scaleFromCenter((kinectROI.width - minborderDist) / kinectROI.width, (kinectROI.height - minborderDist) / kinectROI.height);

	// Try to set a location for the Fish mother outside of the water to be sure the fish cannot reach her without help
	ofVec2f location;
	if (!setRandomVehicleLocation(internalBorders, false, location)) {
		return false;
	}
	motherFish = location;

	// Set the mother Fish plateform location under the sea level
	motherFish.z = kinectProjector->elevationToKinectDepth(-10, motherFish.x, motherFish.y);
	for (auto & f : fish) {
		f.setMotherLocation(motherFish);
	}
	showMotherFish = true;
	return true;
}

bool CBoidGameController::addMotherRabbit() {
	int minborderDist = 40;
	ofRectangle internalBorders = kinectROI;
	internalBorders.scaleFromCenter((kinectROI.width - minborderDist) / kinectROI.width, (kinectROI.height - minborderDist) / kinectROI.height);

	// Set a location for the Rabbits mother inside of the water to be sure the rabbits cannot reach her without help
	ofVec2f location;
	if (!setRandomVehicleLocation(internalBorders, true, location)) {
		return false;
	}
	motherRabbit = location;

	// Set the mother Rabbit plateform location over the sea level
	motherRabbit.z = kinectProjector->elevationToKinectDepth(10, motherRabbit.x, motherRabbit.y);

	for (auto & r : rabbits) {
		r.setMotherLocation(motherRabbit);
	}
	showMotherRabbit = true;
	return true;
}

bool CBoidGameController::setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location) {
	bool okwater = false;
	int count = 0;
	int maxCount = 100;
	while (!okwater && count < maxCount) {
		count++;
		float x = ofRandom(area.getLeft(), area.getRight());
		float y = ofRandom(area.getTop(), area.getBottom());
		bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
		if ((insideWater && liveInWater) || (!insideWater && !liveInWater)) {
			location = ofVec2f(x, y);
			okwater = true;
		}
	}
	return okwater;
}

void CBoidGameController::drawVehicles()
{
	//fboVehicles.begin();
	//ofClear(255, 255, 255, 0);
	if (showMotherFish)
		drawMotherFish();
	if (showMotherRabbit)
		drawMotherRabbit();
	for (auto & f : fish) {
		f.draw();
	}
	for (auto & r : rabbits) {
		r.draw();
	}
	for (auto & s : sharks) {
		s.draw();
	}
	//fboVehicles.end();
}


void CBoidGameController::drawMotherFish()
{
	// Mother fish scale
	float sc = 10;
	float tailSize = 1 * sc;
	float fishLength = 2 * sc;
	float fishHead = tailSize;
	float tailangle = 0;

	ofPushMatrix();
	ofTranslate(kinectProjector->kinectCoordToProjCoord(motherFish.x + tailSize, motherFish.y));

	ofFill();
	ofSetColor(ofColor::blueSteel);
	ofDrawCircle(-0.5*sc, 0, motherPlatformSize);

	ofFill();
	ofSetColor(255);
	ofPolyline fish;
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8)));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle + 0.8), tailSize*sin(tailangle + 0.8)));
	fish.curveTo(ofPoint(-fishLength, 0));
	fish.curveTo(ofPoint(0, -fishHead));
	fish.curveTo(ofPoint(fishHead, 0));
	fish.curveTo(ofPoint(0, fishHead));
	fish.curveTo(ofPoint(-fishLength, 0));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8)));
	fish.curveTo(ofPoint(-fishLength - tailSize*cos(tailangle - 0.8), tailSize*sin(tailangle - 0.8)));
	fish.close();
	ofSetLineWidth(2.0);
	fish.draw();
	ofSetColor(255);
	ofDrawCircle(0, 0, 5);
	ofPopMatrix();
}

void CBoidGameController::drawMotherRabbit()
{
	float sc = 2; // MotherRabbit scale
	ofPushMatrix();
	ofTranslate(kinectProjector->kinectCoordToProjCoord(motherRabbit.x + 5 * sc, motherRabbit.y));

	ofFill();
	ofSetColor(ofColor::green);
	ofDrawCircle(-5 * sc, 0, motherPlatformSize);

	ofFill();
	ofSetLineWidth(1.0);
	ofPath body;
	body.curveTo(ofPoint(-2 * sc, 5.5*sc));
	body.curveTo(ofPoint(-2 * sc, 5.5*sc));
	body.curveTo(ofPoint(-9 * sc, 7.5*sc));
	body.curveTo(ofPoint(-17 * sc, 0 * sc));
	body.curveTo(ofPoint(-9 * sc, -7.5*sc));
	body.curveTo(ofPoint(-2 * sc, -5.5*sc));
	body.curveTo(ofPoint(4 * sc, 0 * sc));
	body.curveTo(ofPoint(4 * sc, 0 * sc));
	body.close();
	body.setFillColor(0);
	body.draw();

	ofSetColor(255);
	ofDrawCircle(-19 * sc, 0, 2 * sc);

	ofPath head;
	head.curveTo(ofPoint(0, 1.5*sc));
	head.curveTo(ofPoint(0, 1.5*sc));
	head.curveTo(ofPoint(-3 * sc, 1.5*sc));
	head.curveTo(ofPoint(-9 * sc, 3.5*sc));
	head.curveTo(ofPoint(0, 5.5*sc));
	head.curveTo(ofPoint(8 * sc, 0));
	head.curveTo(ofPoint(0, -5.5*sc));
	head.curveTo(ofPoint(-9 * sc, -3.5*sc));
	head.curveTo(ofPoint(-3 * sc, -1.5*sc));
	head.curveTo(ofPoint(0, -1.5*sc));
	head.curveTo(ofPoint(0, -1.5*sc));
	head.close();
	head.setFillColor(255);
	head.draw();

	ofSetColor(0);
	ofDrawCircle(8.5*sc, 0, 1 * sc);

	ofPopMatrix();
	ofSetColor(255);
}

void CBoidGameController::setupGui() {
	// instantiate and position the gui //
	gui = new ofxDatGui();
	//	auto animalGame = gui->addFolder("Animal Game", ofColor::greenYellow);

	//animalGame->addSlider("# of fish", 0, 10, fish.size())->setPrecision(0);
	//animalGame->addSlider("# of rabbits", 0, 10, rabbits.size())->setPrecision(0);
	//animalGame->addToggle("Mother fish", showMotherFish);
	//animalGame->addToggle("Mother rabbit", showMotherRabbit);
	//animalGame->addButton("Remove all animals");

//	gui->addButton("Start Sandimal game");
//	gui->addButton("Start Seek Mother game");

	gui->addSlider("# of fish", 0, 200, fish.size())->setPrecision(0);
	gui->addSlider("# of rabbits", 0, 50, rabbits.size())->setPrecision(0);
	gui->addSlider("# of sharks", 0, 10, sharks.size())->setPrecision(0);
	gui->addToggle("Mother fish", showMotherFish);
	gui->addToggle("Mother rabbit", showMotherRabbit);
	gui->addToggle("Draw flipped", doFlippedDrawing);
	gui->addButton("Remove all animals");

	gui->addHeader(":: Games ::", false);

	gui->onButtonEvent(this, &CBoidGameController::onButtonEvent);
	gui->onToggleEvent(this, &CBoidGameController::onToggleEvent);
	gui->onSliderEvent(this, &CBoidGameController::onSliderEvent);
	gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);

	gui->setPosition(ofxDatGuiAnchor::BOTTOM_RIGHT); // You have to do it at the end
	gui->setAutoDraw(false); // troubles with multiple windows drawings on Windows

//	std::cout << "GUI size " << gui->getWidth() << " x " << gui->getHeight() << std::endl;
}

void CBoidGameController::UpdateGUI()
{
	gui->getSlider("# of fish")->setValue(fish.size());
	gui->getSlider("# of rabbits")->setValue(rabbits.size());
	gui->getSlider("# of sharks")->setValue(sharks.size());
	gui->getToggle("Mother fish")->setEnabled(showMotherFish);
	gui->getToggle("Mother rabbit")->setEnabled(showMotherRabbit);
}

bool CBoidGameController::isIdle()
{
	return(GameSequence[CurrentGameSequence] == GAME_STATE_IDLE);
}

void CBoidGameController::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Remove all animals")) {
		fish.clear();
		rabbits.clear();
		sharks.clear();
		showMotherFish = false;
		showMotherRabbit = false;
		gui->getSlider("# of fish")->setValue(0);
		gui->getSlider("# of rabbits")->setValue(0);
		gui->getToggle("Mother fish")->setChecked(false);
		gui->getToggle("Mother rabbit")->setChecked(false);
	}
	//else if (e.target->is("Start Sandimal game"))
	//{
	//	StartGame();
	//}
	//else if (e.target->is("Start Seek Mother game"))
	//{
	//	StartSeekMotherGame();
	//}
}

void CBoidGameController::onToggleEvent(ofxDatGuiToggleEvent e) {
	if (e.target->is("Mother fish")) {
		if (!showMotherFish) {
			if (!addMotherFish())
				e.target->setChecked(false);
		}
		else {
			showMotherFish = e.checked;
		}
	}
	else if (e.target->is("Mother rabbit")) {
		if (!showMotherRabbit) {
			if (!addMotherRabbit())
				e.target->setChecked(false);
		}
		else {
			showMotherRabbit = e.checked;
		}
	}
	else if (e.target->is("Draw flipped")) {
		doFlippedDrawing = e.checked;
	}
}

void CBoidGameController::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("# of fish")) {
		if (e.value > fish.size())
			while (e.value > fish.size()) {
				addNewFish();
			}
		if (e.value < fish.size())
			while (e.value < fish.size()) {
				fish.pop_back();
			}

	}
	else if (e.target->is("# of rabbits")) {
		if (e.value > rabbits.size())
			while (e.value > rabbits.size()) {
				addNewRabbit();
			}
		if (e.value < rabbits.size())
			while (e.value < rabbits.size()) {
				rabbits.pop_back();
			}
	}
	else if (e.target->is("# of sharks")) {
		if (e.value > sharks.size())
			while (e.value > sharks.size()) {
				addNewShark();
			}
		if (e.value < sharks.size())
			while (e.value < sharks.size()) {
				sharks.pop_back();
			}
	}
}
