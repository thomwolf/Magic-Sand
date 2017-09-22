/***********************************************************************
MapGameController.cpp - Controller for a Sandbox mapping / Island game
Copyright (c) 2017  Rasmus R. Paulsen (people.compute.dtu.dk/rapa)

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


#include "MapGameController.h"

#include <string>
//#include <direct.h>

CMapGameController::CMapGameController()
{
	DataBaseDir = "mapGame/";
	setDebug(true);

	int xmin = 0;
	int ymin = 0;
	int xmax = 640;
	int ymax = 480;

	MinSearchAngle = 180 - 45;
	MaxSearchAngle = 180 + 45;

	ROI = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
	referenceMapHandler.Init();
	ShowScore = false;
	ButtonPressTime = -1;

	std::string SplashScreenFname = DataBaseDir + "art/IslandGameSplashScreen.png";
	if (!splashScreen.loadImage(SplashScreenFname))
	{
		ofLogVerbose("CMapGameController()") << "could not read splash screen ";
	}

	CheckedForIsland = 0;
	LastTimeEvent = ofGetElapsedTimef();;
	doShowMatchResultContourLines = true;
	SetupGameSequence();
}

CMapGameController::~CMapGameController()
{
}

void CMapGameController::setup(std::shared_ptr<KinectProjector> const& k)
{
	kinectProjector = k;

	ofTrueTypeFont::setGlobalDpi(72);
	if (!scoreFont.loadFont("verdana.ttf", 64))
		std::cout << "Could not read font verdana.ttf" << std::endl;
	if (!nameFont.loadFont("verdana.ttf", 128))
		std::cout << "Could not read font verdana.ttf" << std::endl;

	//if (!nameFont.loadFont("cooperBlack.ttf", 64))
	//	std::cout << "Could not read font cooperBlack.ttf" << std::endl;

	referenceMapHandler.Init();
	referenceMapHandler.SetCycleMode(2);
	int nRefMaps = referenceMapHandler.ReferenceMaps.size();
	scoreTrackers.resize(nRefMaps);
	scoreFileNames.resize(nRefMaps);

	for (int i = 0; i < nRefMaps; i++)
	{
		std::string tfname = ofToDataPath(DataBaseDir + "Scores/" + referenceMapHandler.ReferenceMaps[i] + "_scores.xml");
		scoreFileNames[i] = tfname;

		scoreTrackers[i].LoadScoresXML(scoreFileNames[i]);
	}
}


void CMapGameController::update()
{
	float resultTime = ofGetElapsedTimef();

	if (ShowScore)
	{
		if (resultTime - LastTimeEvent > 3)
		{
			ShowScore = false;
		}
	}

	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence == GAME_STATE_SHOWMAPNAME)
	{
		// Do nothing in update
	}
	else if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		PlayAndShowCountDown(resultTime);
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT)
	{
		// Do nothing in update
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		// Do nothing in update
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
}

void CMapGameController::PlayAndShowCountDown(int resultTime)
{
	if (ButtonPressTime > 0 && resultTime - ButtonPressTime < 5)
	{
		// Button has been pressed and score is shown
		return;
	}

	ButtonPressTime = -1;

	fboProjWindow.begin();
	ofClear(255, 255, 255, 0);
	ofSetColor(255, 255, 255);
	int DeltaTime = GameSequenceTimings[CurrentGameSequence];
	int timeleft = DeltaTime - (resultTime - LastTimeEvent);

	std::string timeString = ofToString(timeleft);

	double sW = scoreFont.stringWidth(timeString);
	double sH = scoreFont.stringHeight(timeString);

	double sx = projROI.x + (projROI.width - sW - 70);
	double sy = projROI.y + 70;

	scoreFont.drawString(timeString, sx, sy);

	//if (GameSequence[CurrentGameSequence+1] == GAME_STATE_SHOWINTERMIDEATERESULT || 
	//	GameSequence[CurrentGameSequence + 1] == GAME_STATE_SHOWFINALRESULT)
	//{

	//	int currentStep = 0;
	//	int totalSteps = 0;
	//	ComputeWhereInSequence(currentStep, totalSteps);
	//	std::string stepString = "Tries: " + ofToString(currentStep) + " / " + ofToString(totalSteps);
	//	if (GameSequence[CurrentGameSequence + 1] == GAME_STATE_SHOWFINALRESULT)
	//	{
	//		stepString = stepString + " (final)";
	//	}
	//	//sW = nameFont.stringWidth(stepString);
	//	//sH = nameFont.stringHeight(stepString);

	//	sx = projROI.x + 70;
	//	sy = projROI.y + 70;

	//	scoreFont.drawString(stepString, sx, sy);
	//}

	if (timeleft < 4)
	{
		std::string warningString = "Hands Off!";
		double sW = nameFont.stringWidth(warningString);
		double sH = nameFont.stringHeight(warningString);

		double sx = projROI.x + (projROI.width / 2 - sW / 2);
		double sy = projROI.y + (projROI.height / 2 - sH / 2);

		nameFont.drawString(warningString, sx, sy);
	}

	fboProjWindow.end();
}

void CMapGameController::DrawScoresOnFBO()
{
	fboProjWindow.begin();

	if (matchResultImage.getWidth() > 0)
		matchResultImage.draw(0, 0);

	if (doShowMatchResultContourLines)
	{
		DrawMatchResultContourLines();
	}
	DrawScoreTexts();

	fboProjWindow.end();
}

void CMapGameController::DrawScoreTexts()
{
	ofSetColor(10, 30, 200);

	scoreFont.drawString(ScoreText, 100, 100);
	scoreFont.drawString(HiScoreText, 100, projRes.y-100);

}

void CMapGameController::drawProjectorWindow()
{
	if (ShowScore)
	{
		fboProjWindow.draw(0, 0);
	}

	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence == GAME_STATE_SHOWMAPNAME || sequence == GAME_STATE_SHOWSPLASHSCREEN)
	{
		fboProjWindow.draw(0, 0);
	}
	else if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		fboProjWindow.draw(0, 0);
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT || sequence == GAME_STATE_CHECKFORISLAND)
	{
		fboProjWindow.draw(0, 0);
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		fboProjWindow.draw(0, 0);
	}
}


bool  CMapGameController::InitiateGameSequence()
{
	eGameState sequence = GameSequence[CurrentGameSequence];

	if (sequence == GAME_STATE_SHOWMAPNAME)
	{
		CreateWelcomeScreen();
		std::cout << "Created welcome screen" << std::endl;
	}
	else if (sequence == GAME_STATE_SHOWSPLASHSCREEN)
	{
		CreateSplashScreen();
		std::cout << "Created splash screen" << std::endl;
	}
	else if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		fboProjWindow.begin();
		ofClear(255, 255, 255, 0);
		//ofSetColor(255, 255, 25);
		//scoreFont.drawString("Time Left: 60", projRes.x - 200, 50);
		fboProjWindow.end();
		std::cout << "Playing with countdown started" << std::endl;
	}
	else if (sequence == GAME_STATE_CHECKFORISLAND)
	{
		if (!CheckForIsland())
		{
			if (CheckedForIsland++ > 3)
			{
				// stop the game
				CurrentGameSequence = GameSequence.size() - 1; // Set to IDLE by default
			}
			else
			{
				// Go back 
				CurrentGameSequence--;
				CurrentGameSequence--;
			}
		}
		std::cout << "Checked for Island" << std::endl;
	}
	else if (sequence == GAME_STATE_SHOWINTERMIDEATERESULT)
	{
		if (!CreateIntermediateResult())	
		{
			fboProjWindow.begin();
			ofClear(255, 255, 255, 0);
			ofSetColor(255, 255, 255);
			nameFont.drawString("No Island Found!", 100, projRes.y / 2);
			fboProjWindow.end();
		}
		std::cout << "Intermediate result shown" << std::endl;
	}
	else if (sequence == GAME_STATE_SHOWFINALRESULT)
	{
		if (!CreateFinalResult())
		{
			fboProjWindow.begin();
			ofClear(255, 255, 255, 0);
			ofSetColor(255, 255, 255);
			nameFont.drawString("No Island Found!", 100, projRes.y / 2);
			fboProjWindow.end();
		}
		std::cout << "Final result shown" << std::endl;
	}

	LastTimeEvent = ofGetElapsedTimef();
	return true;
}

bool CMapGameController::CreateIntermediateResult()
{
	if (!GetActualBinaryLandImage())
		return false;

	if (!ConnectedComponentAnalysis())
		return false;

	if (!MatchMap())
		return false;

	if (!ComputeProjectorMap())
		return false;

	if (!CreateScoreTexts(true))
		return false;

	DrawScoresOnFBO();

	return true;
}

bool CMapGameController::CheckForIsland()
{
	bool IslandFound = true;

	if (!GetActualBinaryLandImage())
		IslandFound = false;

	if (!ConnectedComponentAnalysis())
		IslandFound = false;


	if (!IslandFound)
	{
		fboProjWindow.begin();

		ofSetColor(255, 255, 255);
		ofClear(255, 255, 255, 0);
//		ofBackground(255);

		std::string warningString = "No Island Found!";
		double sW = nameFont.stringWidth(warningString);
		double sH = nameFont.stringHeight(warningString);

		double sx = projROI.x + (projROI.width / 2 - sW / 2);
		double sy = projROI.y + (projROI.height / 2 - sH / 2);

		nameFont.drawString(warningString, sx, sy);

		fboProjWindow.end();
	}
	else
	{
		fboProjWindow.begin();

		ofSetColor(255, 255, 255);
		ofClear(255, 255, 255, 0);
		
		DrawMatchResultContourLines();

		CheckedForIsland = 0;

		fboProjWindow.end();
	}

	return IslandFound;
}

bool CMapGameController::CreateFinalResult()
{
	if (!GetActualBinaryLandImage())
		return false;

	if (!ConnectedComponentAnalysis())
		return false;

	if (!MatchMap())
		return false;

	if (!ComputeProjectorMap())
		return false;

	if (!CreateScoreTexts(false))
		return false;

	DrawScoresOnFBO();

	if (!UpdateScores())
		return false;
	return true;
}

bool CMapGameController::CreateWelcomeScreen()
{
	int actRef = referenceMapHandler.GetActualRef();
	std::string RefMapName = referenceMapHandler.ReferenceNames[actRef];

	std::string HScoreText;

	bool isThereHighScore = false;
	int currentHScore = 0;

	std::string HscoreFname;
	if (scoreTrackers[actRef].getHighScore(currentHScore, HscoreFname) && currentHScore > 0)
	{
		isThereHighScore = true;
	}

	if (!isThereHighScore)
	{
		HScoreText = "No High-Score";
	}
	else
	{
		HScoreText = "High-Score: " + ofToString(currentHScore);
	}
	bool DrawRefImage = false;

	fboProjWindow.begin();
	ofClear(255, 255, 255, 0);
	ofBackground(255);

	// Where to draw map Splash image
	ofRectangle splashROI;
	splashROI.width = projROI.getWidth() * 0.90;
	splashROI.height = projROI.getHeight() * 0.90;
	splashROI.x = projROI.x + (projROI.width / 2 - splashROI.width / 2);
	splashROI.y = projROI.y + (projROI.height / 2 - splashROI.height / 2);

	ofRectangle toptextROI;
	toptextROI.width = projROI.getWidth() * 0.90;
	toptextROI.height = projROI.getHeight()  * 0.90 / 4;
	toptextROI.x = projROI.x;
	toptextROI.y = projROI.y;

	ofRectangle bottomtextROI;
	bottomtextROI.width = projROI.getWidth() * 0.90;
	bottomtextROI.height = projROI.getHeight()* 0.90 / 4;
	bottomtextROI.x = projROI.x;
	bottomtextROI.y = projROI.y +  projROI.getHeight() / 2;


	if (DrawRefImage)
	{

		int actRef = referenceMapHandler.GetActualRef();
		std::string RefMap = referenceMapHandler.ReferenceMaps[actRef];
		std::string RefMapName = referenceMapHandler.ReferenceNames[actRef];

		std::string referenceImage = DataBaseDir + "ReferenceData/" + RefMap + "_Splash.png";

		ofImage temp;
		if (!temp.loadImage(referenceImage))
		{
			ofLogVerbose("CMapGameController") << "CreateWelcomeScreen(): could not read " + referenceImage;
			return false;
		}
		double sW = temp.getWidth();
		double sH = temp.getHeight();
		if (sW == 0 || sH == 0)
		{
			ofLogVerbose("CMapGameController") << "splash image invalid";
			return false;
		}

		double scale = splashROI.getHeight() / sH;
		double sWn = scale * sW;
		double sHn = splashROI.getHeight();

		if (sWn > splashROI.getWidth())
		{
			scale = splashROI.getWidth() / sW;
			sHn = scale * sH;
			sWn = splashROI.getWidth();
		}

		temp.draw(splashROI.x, splashROI.y, sWn, sHn);
	}

	ofSetColor(10, 30, 200);
	//ofSetColor(255, 255, 255);
//	nameFont.drawString(RefMapName, 200, projRes.y / 2);

	std::string mapString = "Now Shape:";
	double sW = nameFont.stringWidth(mapString);
	double sH = nameFont.stringHeight(mapString);

	double sx = projROI.x + (projROI.width / 2 - sW / 2);
	double sy = projROI.y + (projROI.height / 2 - sH * 1.2);

	//// Center in top text roi
	//double sx = toptextROI.x + (toptextROI.width / 2 - sW / 2);
	//double sy = toptextROI.y + (toptextROI.height / 2 - sH / 2);

	nameFont.drawString(mapString, sx, sy);

	mapString = RefMapName;
	sW = nameFont.stringWidth(mapString);
	sH = nameFont.stringHeight(mapString);

	//sx = bottomtextROI.x + (bottomtextROI.width / 2 - sW / 2);
	//sy = bottomtextROI.y + (bottomtextROI.height / 2 - sH / 2);

	sx = projROI.x + (projROI.width / 2 - sW / 2);
	sy = projROI.y + (projROI.height / 2 + sH * 1.2);

	nameFont.drawString(mapString, sx, sy);

	sW = scoreFont.stringWidth(HScoreText);
	sH = scoreFont.stringHeight(HScoreText);
	//sx = bottomtextROI.width - sW - 80;
	//sy = bottomtextROI.y + (bottomtextROI.height / 2 - sH / 2);
	sx = projROI.x + (projROI.width / 2 - sW / 2);
	//sx = projROI.width - sW - 80;
	sy = projROI.height - sH - 80;

	scoreFont.drawString(HScoreText, sx, sy);

	fboProjWindow.end();

	return true;
}

bool CMapGameController::CreateSplashScreen()
{
	CheckedForIsland = 0;

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

	fboProjWindow.begin();
	ofClear(255, 255, 255, 0);
	ofBackground(255);

	splashScreen.draw(xs, ys, sWn, sHn);
	//ofTrueTypeFont HeadingFont;
	//if (!HeadingFont.loadFont("cooperBlack.ttf", 128))
	//{
	//	std::cout << "Could not read font cooperBlack.ttf" << std::endl;
	//	return false;
	//}

	//std::string HeadText = "The Island Game";
	//double sW = HeadingFont.stringWidth(HeadText);
	//double sH = HeadingFont.stringHeight(HeadText);

	//

	////ofSetColor(31, 78, 121);
	////nameFont.drawString(RefMapName, 200, projRes.y / 2);
	////scoreFont.drawString(HScoreText, 100, projRes.y - 100);

	fboProjWindow.end();
	return true;
}

bool CMapGameController::StartGame()
{
	if (GameSequence[CurrentGameSequence] != GAME_STATE_IDLE)
	{
		// Stop the game
		CurrentGameSequence = GameSequence.size() - 1; // Set to IDLE by default
		return false;
	}

	referenceMapHandler.CycleMap();
	projROI = kinectProjector->getProjectorActiveROI();
	CurrentGameSequence = 0;
	InitiateGameSequence();

	return true;
}

bool CMapGameController::ButtonPressed()
{
	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
	{
		float resultTime = ofGetElapsedTimef();

		int DeltaTime = GameSequenceTimings[CurrentGameSequence];
		int timeleft = DeltaTime - (resultTime - LastTimeEvent);

		if (timeleft > 12)
		{
			if (!CreateIntermediateResult())
			{
				fboProjWindow.begin();
				ofClear(255, 255, 255, 0);
				ofSetColor(255, 255, 255);
				nameFont.drawString("No Island Found!", 100, projRes.y / 2);
				fboProjWindow.end();
			}

			// Decrease time left
			LastTimeEvent -= 8;
			ButtonPressTime = resultTime;
		}
	}
	return true;
}

bool CMapGameController::EndButtonPressed()
{
	CurrentGameSequence++;
	// Debug check
	if (CurrentGameSequence >= GameSequence.size())
		CurrentGameSequence = GameSequence.size() - 1;

	InitiateGameSequence();
	return true;
}

void CMapGameController::TestMe()
{
	ConnectedComponentAnalysis();
	MatchMap();
	ComputeProjectorMap();
}

void CMapGameController::RealTimeTestMe()
{
	if (!GetActualBinaryLandImage())
		return;

	if (!ConnectedComponentAnalysis())
		return;

	if (!MatchMap())
		return;

	if (!ComputeProjectorMap())
		return;

	if (!CreateScoreTexts(false))
		return;

	DrawScoresOnFBO();

	if (!UpdateScores())
		return;

	ShowScore = true;
	LastTimeEvent = ofGetElapsedTimef();
}

void CMapGameController::DebugTestMe()
{
	if (!ConnectedComponentAnalysis())
		return;

	if (!MatchMap())
		return;

	if (!ComputeProjectorMap())
		return;

	//if (!CreateScoreTexts(false))
	//	return;

	//DrawScoresOnFBO();

	//if (!UpdateScores())
	//	return;

	//ShowScore = true;
	//LastTimeEvent = GetTimeStamp();

}

bool CMapGameController::isIdle()
{
	return(GameSequence[CurrentGameSequence] == GAME_STATE_IDLE);
}


void CMapGameController::setProjectorRes(ofVec2f& PR)
{
	projRes = PR;

	fboProjWindow.allocate(projRes.x, projRes.y, GL_RGBA);
	fboProjWindow.begin();
	ofClear(255, 255, 255, 0);
	ofBackground(255); // Set to white in setup mode
	fboProjWindow.end();
}

void CMapGameController::setKinectRes(ofVec2f& KR)
{
	kinectRes = KR;
}

void CMapGameController::setKinectROI(ofRectangle &KROI)
{
	kinectROI = KROI;
	ROI.x = kinectROI.x;
	ROI.y = kinectROI.y;
	ROI.width = kinectROI.width;
	ROI.height = kinectROI.height;
}

ofRectangle CMapGameController::getKinectROI()
{
	return kinectROI;
}

void CMapGameController::setDebug(bool flag)
{
	debugOn = flag;

	if (flag)
	{
		std::string BaseDir = "data//mapGame//DebugFiles//";
        // TODO: Find a way to create folder both on Win/Mac
        //		mkdir(BaseDir.c_str());

		// By default the application looks in data
		debugBaseDir = "mapGame//DebugFiles//";
	}
}

bool CMapGameController::GetActualBinaryLandImage()
{
	ofLogVerbose("GetActualDepthImage") << "getting raw land data ";

	std::string BinOutName = debugBaseDir + "BinaryLandImage.png";
	
	if (!kinectProjector->getBinaryLandImage(BinaryLandImage))
		return false;

	if (debugOn)
	{
		ofSaveImage(BinaryLandImage.getPixels(), BinOutName);
	}
	
	ofLogVerbose("GetActualDepthImage") << "done getting raw land ";
	return true;
}

void CMapGameController::RemoveBorderBlobs(cv::Mat& image)
{
	const uchar white(255);

	// do top and bottom row
	for (int y = 0; y < image.rows; y += image.rows - 1)
	{
		uchar* row = image.ptr<uchar>(y);
		for (int x = 0; x < image.cols; ++x)
		{
			if (row[x] == white)
			{
				cv::floodFill(image, cv::Point(x, y), cv::Scalar(0), (cv::Rect*)0, cv::Scalar(), cv::Scalar(200));
			}
		}
	}
	// fix left and right sides
	for (int y = 0; y < image.rows; ++y)
	{
		uchar* row = image.ptr<uchar>(y);
		for (int x = 0; x < image.cols; x += image.cols - 1)
		{
			if (row[x] == white)
			{
				cv::floodFill(image, cv::Point(x, y), cv::Scalar(0), (cv::Rect*)0, cv::Scalar(), cv::Scalar(200));
			}
		}
	}
}

bool CMapGameController::WriteGreyOpenCVMat(cv::Mat &img, const std::string &fname)
{
	cv::Mat t;
	img.copyTo(t);  // Do a temporary copy to avoid trouble with ROI definitions and data boundaries

	unsigned char *input = (unsigned char*)(t.data);
	ofxCvGrayscaleImage img2;
	img2.setFromPixels(input, t.cols, t.rows);
	ofSaveImage(img2.getPixels(), fname);
	return true;
}

bool CMapGameController::WriteRGBOpenCVMat(cv::Mat &img, const std::string &fname)
{
	cv::Mat t;
	img.copyTo(t);  // Do a temporary copy to avoid trouble with ROI definitions and data boundaries

	unsigned char *input = (unsigned char*)(t.data);
	ofxCvColorImage img2;
	img2.setFromPixels(input, t.cols, t.rows);
	ofSaveImage(img2.getPixels(), fname);
	return true;
}


void CMapGameController::SetupGameSequence()
{
	GameSequence.push_back(GAME_STATE_SHOWSPLASHSCREEN);
	GameSequenceTimings.push_back(3);

	GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	GameSequenceTimings.push_back(30);

	GameSequence.push_back(GAME_STATE_CHECKFORISLAND);
	GameSequenceTimings.push_back(3);

	GameSequence.push_back(GAME_STATE_SHOWMAPNAME);
	GameSequenceTimings.push_back(3);

	GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	GameSequenceTimings.push_back(180);

	//GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	//GameSequenceTimings.push_back(5);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(60);

	//GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	//GameSequenceTimings.push_back(5);

	////GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	////GameSequenceTimings.push_back(20);

	////GameSequence.push_back(GAME_STATE_SHOWINTERMIDEATERESULT);
	////GameSequenceTimings.push_back(5);

	//GameSequence.push_back(GAME_STATE_PLAYANDSHOWCOUNTDOWN);
	//GameSequenceTimings.push_back(30);

	GameSequence.push_back(GAME_STATE_SHOWFINALRESULT);
	GameSequenceTimings.push_back(5);

	GameSequence.push_back(GAME_STATE_IDLE);
	GameSequenceTimings.push_back(-1); // Indefinite

	CurrentGameSequence = GameSequence.size()-1; // Set to IDLE by default
}

bool CMapGameController::ComputeWhereInSequence(int &currentStep, int &totalSteps)
{
	totalSteps = 0;
	currentStep = 0;
	for (int i = 0; i < GameSequence.size(); i++)
	{
		if (GameSequence[i] == GAME_STATE_PLAYANDSHOWCOUNTDOWN)
		{
			totalSteps++;
			if (i == CurrentGameSequence)
			{
				currentStep = totalSteps;
			}
		}
	}
	return true;
}

bool CMapGameController::ConnectedComponentAnalysis()
{
	std::string iThres = debugBaseDir + "BinaryLandImage.png";
//	std::string iROI = debugBaseDir + "LandImageROI.txt";

	std::string onameRoiPng = debugBaseDir + "HeightMap_ROI.png";
	std::string oNoBord = debugBaseDir + "HM_cutout_noBorderBlobs.png";
//	std::string oLabels = debugBaseDir + "HM_cutout_labels.png";
	std::string oMaxBlob = debugBaseDir + "HM_cutout_maxBlob.png";
//	std::string oMaxBlobCont = debugBaseDir + "HM_cutout_maxBlob_contours.png";

	cv::Mat M;
	if (BinaryLandImage.width == 0)
	{
		if (!SafeReadBinaryImage(M, iThres))
		{
				ofLogVerbose("CMapGameController") << "ConnectedComponentAnalysis(): could not read BinaryLandImage.png";
				return false;
		}
	}
	else
	{
		M = ofxCv::toCv(BinaryLandImage);
	}

	cv::Mat bw = M(ROI);

	if (debugOn)
	{
		WriteGreyOpenCVMat(bw, onameRoiPng);
	}

	RemoveBorderBlobs(bw);

	if (debugOn)
	{
		WriteGreyOpenCVMat(bw, oNoBord);
	}

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (contours.size() == 0)
	{
		ofLogVerbose("CMapGameController") << "ConnectedComponentAnalysis(): could not find any contours";
		return false;
	}
	int MaxCont = 0;
	int maxID = -1;

	for (int i = 0; i < contours.size(); i++)
	{
		int sz = contours[i].size();
		if (sz > MaxCont)
		{
			MaxCont = sz;
			maxID = i;
		}
	}
	ofLogVerbose("CMapGameController") << "Found max contour with " + ofToString(MaxCont) + " points";

	maxBLOBImage = cv::Mat::zeros(bw.size(), CV_8UC1);
	drawContours(maxBLOBImage, contours, maxID, 255,  CV_FILLED, 8); // Draws as a filled image

	if (debugOn)
	{
		WriteGreyOpenCVMat(maxBLOBImage, oMaxBlob);
	}

	double BLOBArea = cv::sum(maxBLOBImage)[0] / 255.0;
	double MaxPossible = kinectROI.width * kinectROI.height;
	double occupPercent = BLOBArea / MaxPossible;
	if (occupPercent < 0.05)
	{
		ofLogVerbose("CMapGameController") << "ConnectedComponentAnalysis(): Max BLOB area too small";
		return false;
	}


	MatchResultContours.clear();

	// Store contours in projector coordinates
	for (int i = 0; i < contours[maxID].size(); i++)
	{
		cv::Point pc = contours[maxID][i];
		pc.x += kinectROI.x;
		pc.y += kinectROI.y;

		ofVec2f tp = kinectProjector->kinectCoordToProjCoord(pc.x, pc.y);
		//tp.y = projRes.y - tp.y;

		MatchResultContours.push_back(tp);
	}

	return true;
}


void CMapGameController::ComputeLandmarksFromTemplate(cv::Mat& I, std::vector<cv::Point2f> &LMs)
{
	LMs.clear();

	int maxx = 0;
	int minx = I.cols;
	int maxy = 0;
	int miny = I.rows;

	// Find template size
	for (int i = 0; i < I.rows; ++i)
	{
		const uchar *p = I.ptr<uchar>(i);
		for (int j = 0; j < I.cols; ++j)
		{
			if (p[j] > 0)
			{
				if (i > maxy)
					maxy = i;
				if (i < miny)
					miny = i;
				if (j > maxx)
					maxx = j;
				if (j < minx)
					minx = j;
			}
		}
	}

	// Squeeze it 20% not to get extreme points
	double W = maxx - minx;
	double H = maxy - miny;
	maxx = minx + W * 0.80;
	minx = minx + W * 0.20;
	maxy = miny + H * 0.80;
	miny = miny + H * 0.20;

	int nx = 5;
	int ny = 5;

	for (int x = 0; x < nx; x++)
	{
		double LMx = (double)minx + x / (double)(nx - 1) * (double)(maxx - minx);
		for (int y = 0; y < ny; y++)
		{
			double LMy = (double)miny + y / (double)(ny - 1) * (double)(maxy - miny);

			LMs.push_back(cv::Point2f(LMx, LMy));
		}
	}
}

bool  CMapGameController::CheckCollinear(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3)
{
	cv::Point2f a = p2 - p1;
	cv::Point2f b = p3 - p1;
	double la = sqrt(a.x * a.x + a.y * a.y);
	double lb = sqrt(b.x * b.x + b.y * b.y);
	if (la == 0 || lb == 0)
		return true;

	a.x = a.x / la;
	a.y = a.y / la;
	b.x = b.x / lb;
	b.y = b.y / lb;
	double Dot = abs(a.dot(b));
	if (Dot > 0.98)
		return true;

	return false;
}

bool CMapGameController::SelectFourGoodLms(std::vector<cv::Point2f> &LMs, std::vector<int> &goodIds)
{
	int xmin = ROI.x;
	int ymin = ROI.y;
	int xmax = ROI.x + ROI.width;
	int ymax = ROI.y + ROI.height;

	// Select first LM
	bool found = false;
	for (int i = 0; i < LMs.size() && !found; i++)
	{
		if (LMs[i].x >= xmin && LMs[i].x < xmax && LMs[i].y >= ymin && LMs[i].y < ymax)
		{
			goodIds.push_back(i);
			found = true;
		}
	}
	if (!found)
	{
		std::cerr << "Could not find 1. landmark" << std::endl;
		return false;
	}

	// Select second LM - some distance from LM 1
	double maxD = 5;
	int idx = -1;
	for (int i = 0; i < LMs.size(); i++)
	{
		if (i != goodIds[0])
		{
			if (LMs[i].x >= xmin && LMs[i].x < xmax && LMs[i].y >= ymin && LMs[i].y < ymax)
			{
				double D = cv::norm(LMs[goodIds[0]] - LMs[i]);
				if (D > maxD)
				{
					idx = i;
					maxD = D;
				}
			}
		}
	}
	if (idx == -1)
	{
		std::cerr << "Could not find 2. landmark" << std::endl;
		return false;
	}
	else
	{
		goodIds.push_back(idx);
	}


	// Select third LM - some distance from LM 1 and 2 and not collinear
	maxD = 5;
	idx = -1;
	for (int i = 0; i < LMs.size(); i++)
	{
		if (i != goodIds[0] && i != goodIds[1])
		{
			if (LMs[i].x >= xmin && LMs[i].x < xmax && LMs[i].y >= ymin && LMs[i].y < ymax)
			{
				double D1 = cv::norm(LMs[goodIds[0]] - LMs[i]);
				double D2 = cv::norm(LMs[goodIds[1]] - LMs[i]);
				double Dmin = std::min(D1, D2);
				if (Dmin > maxD)
				{
					if (!CheckCollinear(LMs[goodIds[0]], LMs[goodIds[1]], LMs[i]))
					{
						idx = i;
						maxD = Dmin;
					}
				}
			}
		}
	}
	if (idx == -1)
	{
		std::cerr << "Could not find 3. landmark" << std::endl;
		return false;
	}
	else
	{
		goodIds.push_back(idx);
	}

	// Select fourth LM - some distance from LM 1, 2 and 3 and not collinear with any combi
	maxD = 5;
	idx = -1;
	for (int i = 0; i < LMs.size(); i++)
	{
		if (i != goodIds[0] && i != goodIds[1] && i != goodIds[2])
		{
			if (LMs[i].x >= xmin && LMs[i].x < xmax && LMs[i].y >= ymin && LMs[i].y < ymax)
			{
				double D1 = cv::norm(LMs[goodIds[0]] - LMs[i]);
				double D2 = cv::norm(LMs[goodIds[1]] - LMs[i]);
				double D3 = cv::norm(LMs[goodIds[2]] - LMs[i]);
				double Dmin = std::min(D1, D2);
				Dmin = std::min(Dmin, D3);
				if (Dmin > maxD)
				{
					if (!CheckCollinear(LMs[goodIds[0]], LMs[goodIds[1]], LMs[i]) &&
						!CheckCollinear(LMs[goodIds[0]], LMs[goodIds[2]], LMs[i]) &&
						!CheckCollinear(LMs[goodIds[1]], LMs[goodIds[2]], LMs[i]))
					{
						idx = i;
						maxD = Dmin;
					}
				}
			}
		}
	}
	if (idx == -1)
	{
		std::cerr << "Could not find 4. landmark" << std::endl;
		return false;
	}
	else
	{
		goodIds.push_back(idx);
	}

	return true;
}



bool CMapGameController::ComputeCM(const cv::Mat& I, double& CMx, double& CMy)
{
	CMx = 0;
	CMy = 0;
	int N = 0;

	// Compute CM
	for (int i = 0; i < I.rows; ++i)
	{
		const uchar *p = I.ptr<uchar>(i);
		for (int j = 0; j < I.cols; ++j)
		{
			if (p[j] > 0)
			{
				CMx += j;
				CMy += i;
				N++;
			}
		}
	}
	if (N == 0)
		return false;

	CMx /= N;
	CMy /= N;

	return true;
}

bool CMapGameController::ReferenceToCM(cv::Mat& I, cv::Mat& Iout, double &rCMx, double &rCMy)
{
	rCMx = 0;
	rCMy = 0;

	if (!ComputeCM(I, rCMx, rCMy))
	{
		std::cout << "Could not compute CM" << std::endl;
		return false;
	}

	double midX = I.cols / 2;
	double midY = I.rows / 2;

//	std::cout << "CM: " << rCMx << ", " << rCMy << " mid: " << midX << ", " << midY << std::endl;

	double tX = midX - rCMx;
	double tY = midY - rCMy;

	cv::Mat trans_mat = cv::Mat::zeros(2, 3, CV_32FC1);
	trans_mat.at<float>(0, 0) = 1;
	trans_mat.at<float>(1, 1) = 1;
	trans_mat.at<float>(0, 2) = tX;
	trans_mat.at<float>(1, 2) = tY;

	warpAffine(I, Iout, trans_mat, Iout.size());
	return true;
}

bool CMapGameController::SafeReadBinaryImage(cv::Mat &img, const std::string &fname)
{
	ofImage temp;
	if (!temp.loadImage(fname))
	{
		ofLogVerbose("CMapGameController") << "SafeReadBinaryImage(): could not read " + fname;
		return false;
	}
	cv::Mat t1 = ofxCv::toCv(temp.getPixels());
	
	cv::Mat t2; 
	if (t1.channels() > 1)
		cv::cvtColor(t1, t2, cv::COLOR_BGR2GRAY);
	else
		t1.copyTo(t2);
	
	cv::normalize(t2, img, 0, 255, cv::NORM_MINMAX);
	return true;
}

void CMapGameController::CompareImages(cv::Mat& I, cv::Mat& T, int Xi, int Yi, cv::Mat& MatchImage, double &DSC)
{
	int dX = (Xi - T.cols / 2);
	int dY = (Yi - T.rows / 2);

	// Run through template

	int match = 0;
	int Inm = 0;
	int Tnm = 0;
	int NI = 0;
	int NT = 0;
	for (int i = 0; i < T.rows; ++i)
	{
		int Ii = i + dY;
		if (Ii >= 0 && Ii < I.rows)
		{
			uchar *pI = I.ptr<uchar>(Ii);
			uchar *pT = T.ptr<uchar>(i);
			for (int j = 0; j < T.cols; ++j)
			{
				int Ij = j + dX;
				if (Ij >= 0 && Ij < I.cols)
				{
					cv::Vec3b &MarkPixel = MatchImage.at<cv::Vec3b>(Ii, Ij);

					bool Ion = pI[Ij] > 0;
					bool Ton = pT[j] > 0;

					if (Ion)
						NI++;
					if (Ton)
						NT++;

					if (Ion && Ton)
					{
						match++;
						MarkPixel = cv::Vec3b(0, 255, 0);
					}

					if (Ion && !Ton)
					{
						Inm++;
						MarkPixel = cv::Vec3b(128, 0, 0);
					}

					if (!Ion && Ton)
					{
						Tnm++;
						MarkPixel = cv::Vec3b(0, 0, 128);
					}

				}
			}
		}
	}
	DSC = 2 * (double)match / (NI + NT);
}


bool CMapGameController::MatchMap()
{
	int actRef = referenceMapHandler.GetActualRef();
	std::string RefMap = referenceMapHandler.ReferenceMaps[actRef];

	std::string FoundMap = debugBaseDir + "HM_cutout_maxBlob.png";

	std::string referenceMap = DataBaseDir + "ReferenceData/" + RefMap + "_GT.png";
	std::string referenceImage = DataBaseDir + "ReferenceData/" + RefMap + ".png";
	std::string ScaledTransRef = debugBaseDir + RefMap + "_GT_scaled_trans.png";
	std::string BestMatchOut = debugBaseDir + "BestMatchImage.png";
	std::string LMsinOrgDepthOut = ofToDataPath(debugBaseDir + "LMsinRawDepthImage.txt");
	std::string LMsinRefOut = ofToDataPath(debugBaseDir + "LMsinReferenceImage.txt");

	cv::Mat fMap;
	if (BinaryLandImage.width == 0)
	{
		if (!SafeReadBinaryImage(fMap, FoundMap))
			return false;
	}
	else
	{
		fMap = maxBLOBImage;
	}

	cv::Mat refMap;
	if (!SafeReadBinaryImage(refMap, referenceMap))
		return false;

	std::vector<cv::Point2f> AutoLMs;
	ComputeLandmarksFromTemplate(refMap, AutoLMs);

	double AfMap = cv::sum(fMap)[0] / 255.0;
	double ArefMap = cv::sum(refMap)[0] / 255.0;

	std::cout << "Area found " << AfMap << " ref " << ArefMap << std::endl;
	double factor = sqrt(AfMap / ArefMap);

	cv::Mat ScaledI;
	cv::resize(refMap, ScaledI, cv::Size(), factor, factor, cv::INTER_AREA);
//	cv::imwrite(ScaledRef, ScaledI);

	// Apply scaling transformation to LM
	cv::Mat TMat = cv::Mat::zeros(3, 3, CV_32FC1);
	TMat.at<float>(0, 0) = factor;
	TMat.at<float>(1, 1) = factor;
	TMat.at<float>(2, 2) = 1;
	std::vector<cv::Point2f> LMsScaled(AutoLMs.size());
	cv::perspectiveTransform(AutoLMs, LMsScaled, TMat);

	cv::Mat Itrans = cv::Mat::zeros(ScaledI.rows, ScaledI.cols, ScaledI.type());

	double rCMx = 0;
	double rCMy = 0;
	if (!ReferenceToCM(ScaledI, Itrans, rCMx, rCMy))
		return false;

	if (debugOn)
	{
		WriteGreyOpenCVMat(Itrans, ScaledTransRef);
	}

	// Translate landmarks
	double tX = ScaledI.cols / 2 - rCMx;
	double tY = ScaledI.rows / 2 - rCMy;
	TMat.at<float>(0, 0) = 1;
	TMat.at<float>(1, 1) = 1;
	TMat.at<float>(0, 2) = tX;
	TMat.at<float>(1, 2) = tY;
	std::vector<cv::Point2f> LMsScaledTrans(AutoLMs.size());
	cv::perspectiveTransform(LMsScaled, LMsScaledTrans, TMat);


	double CMx = 0;
	double CMy = 0;
	if (!ComputeCM(fMap, CMx, CMy))
	{
		std::cout << "Could not compute CM" << std::endl;
		return false;
	}
	double midX = Itrans.cols / 2;
	double midY = Itrans.rows / 2;
	//logout << "CM: " << CMx << ", " << CMy << " mid: " << midX << ", " << midY << std::endl;

	cv::Mat MatchImage = cv::Mat::zeros(fMap.size(), CV_8UC3);

	double DSC = 0;

	double maxDSC = 0;
	double maxAngle = -1000;
	//double StartAngle = -45;
	//double EndAngle = 45;
// 	double StartAngle = MinSearchAngle;
// 	double EndAngle = MaxSearchAngle;

	// Check rotations
	for (double angle = MinSearchAngle; angle <= MaxSearchAngle; angle++)
	{
		cv::Mat Irot = cv::Mat::zeros(Itrans.rows, Itrans.cols, Itrans.type());

		cv::Point center = cv::Point(Itrans.cols / 2, Itrans.rows / 2);
		double scale = 1.0;
		cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, scale);
		warpAffine(Itrans, Irot, rotMat, Irot.size());

		CompareImages(fMap, Irot, CMx, CMy, MatchImage, DSC);

		if (DSC > maxDSC)
		{
			maxDSC = DSC;
			maxAngle = angle;
		}
		//		std::cout << "Angle " << angle << " DSC " << DSC << std::endl;
	}
	std::cout << "Max DSC " << maxDSC << " at angle " << maxAngle << std::endl;

	double MinScore = 0.70;
	double MaxScore = 1;
	double finalScore = (maxDSC - MinScore) / (MaxScore - MinScore);
	finalScore = std::max(0.0, finalScore);
	Score = finalScore * 1000;


	//std::ofstream fost(ScoreOut.c_str());
	//fost << finalScore << std::endl;

	cv::Mat Irot = cv::Mat::zeros(Itrans.rows, Itrans.cols, Itrans.type());

	cv::Point center = cv::Point(Itrans.cols / 2, Itrans.rows / 2);
	double scale = 1.0;
	cv::Mat rotMat = cv::getRotationMatrix2D(center, maxAngle, scale);
	warpAffine(Itrans, Irot, rotMat, Irot.size());

	if (debugOn)
	{
		MatchImage = cv::Mat::zeros(fMap.size(), CV_8UC3);
		CompareImages(fMap, Irot, CMx, CMy, MatchImage, DSC);
		WriteRGBOpenCVMat(MatchImage, BestMatchOut);
	}
	//cv::imwrite(BestMatchOut, MatchImage);

	//// Rotate landmarks. Copy rotate matrix - since the dimensions do not match TMat
	TMat = cv::Mat::zeros(3, 3, CV_32FC1);
	TMat.at<float>(2, 2) = 1;
	for (int c = 0; c < 3; c++)
	{
		for (int r = 0; r < 2; r++)
		{
			TMat.at<float>(r, c) = rotMat.at<double>(r, c);
		}
	}
	std::vector<cv::Point2f> LMsScaledTransRot(AutoLMs);
	cv::perspectiveTransform(LMsScaledTrans, LMsScaledTransRot, TMat);

	// Translate landmarks to match CM of target
	tX = CMx - ScaledI.cols / 2;
	tY = CMy - ScaledI.rows / 2;
	TMat = cv::Mat::zeros(3, 3, CV_32FC1);
	TMat.at<float>(2, 2) = 1;
	TMat.at<float>(0, 0) = 1;
	TMat.at<float>(1, 1) = 1;
	TMat.at<float>(0, 2) = tX;
	TMat.at<float>(1, 2) = tY;
	std::vector<cv::Point2f> LMsScaledTransRotT2(AutoLMs);
	cv::perspectiveTransform(LMsScaledTransRot, LMsScaledTransRotT2, TMat);

	//	cv::Mat finalTrans = cv::getPerspectiveTransform(LMs, LMsScaledTransRotT2);

	//logout << "reading " << referenceImage << std::endl;
	//cv::Mat RefImage;
	//RefImage = cv::imread(referenceImage);
	//if (!RefImage.data)
	//{
	//	logout << "No image data" << std::endl;
	//	return;
	//}

	//cv::Mat IFinalOut = cv::Mat::zeros(MatchImage.rows, MatchImage.cols, RefImage.type());
	//warpPerspective(RefImage, IFinalOut, finalTrans, IFinalOut.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Vec3f(255, 233, 191));
	//cv::imwrite(MatchRefOut, IFinalOut);

	// Finally find the position of the LM in the original depth image
	// Translate landmarks
	int xmin = ROI.x;
	int ymin = ROI.y;

	TMat.at<float>(0, 0) = 1;
	TMat.at<float>(1, 1) = 1;
	TMat.at<float>(0, 2) = xmin;
	TMat.at<float>(1, 2) = ymin;
	std::vector<cv::Point2f> LMsinOrgDepthImage(AutoLMs);
	cv::perspectiveTransform(LMsScaledTransRotT2, LMsinOrgDepthImage, TMat);


	std::vector<int> goodIds;
	if (!SelectFourGoodLms(LMsinOrgDepthImage, goodIds))
	{
		std::cout << "Something wrong with transformed landmarks" << std::endl;
		return false;
	}



	//std::ofstream fost2(LMsinOrgDepthOut.c_str());
	//for (int i = 0; i < LMsinOrgDepthImage.size(); i++)
	//{
	//	fost2 << LMsinOrgDepthImage[i].x << " " << LMsinOrgDepthImage[i].y << std::endl;
	//}

	LMDepthImage.resize(goodIds.size());
	LMReferenceImage.resize(goodIds.size());

	for (int i = 0; i < goodIds.size(); i++)
	{
		LMDepthImage[i] = LMsinOrgDepthImage[goodIds[i]];
		LMReferenceImage[i] = AutoLMs[goodIds[i]];
	}


	if (debugOn)
	{
		std::ofstream fost2(LMsinOrgDepthOut.c_str());
		std::ofstream fost3(LMsinRefOut.c_str());
		for (int i = 0; i < goodIds.size(); i++)
		{
			fost2 << LMsinOrgDepthImage[goodIds[i]].x << " " << LMsinOrgDepthImage[goodIds[i]].y << std::endl;
			fost3 << AutoLMs[goodIds[i]].x << " " << AutoLMs[goodIds[i]].y << std::endl;
		}
	}

	return true;
}

bool ReadReferenceLMRaw(std::vector<cv::Point2f>& LMs, std::string& referenceLM)
{
	std::ifstream fist(referenceLM.c_str());
	if (!fist)
	{
		std::cerr << "Could not read " << referenceLM << std::endl;
		return false;
	}
	int nLMS = 4;
	LMs.resize(nLMS);

	for (int i = 0; i < nLMS; i++)
	{
		fist >> LMs[i].x >> LMs[i].y;
	}
	return true;
}

bool CMapGameController::EstimateBorderColor(cv::Mat& RefImage, cv::Scalar& borderColor)
{
	double Rsum = 0;
	double Gsum = 0;
	double Bsum = 0;
	int npix = 0;

	for (int y = 0; y < RefImage.rows; y++)
	{
		{
			int x = 0;
			// BGR order
			uchar b = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 0];
			uchar g = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 1];
			uchar r = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 2];

			Rsum += r;
			Gsum += g;
			Bsum += b;
			npix++;
		}
		{
			int x = RefImage.cols-1;
			// BGR order
			uchar b = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 0];
			uchar g = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 1];
			uchar r = RefImage.data[RefImage.channels()*(RefImage.cols*y + x) + 2];

			Rsum += r;
			Gsum += g;
			Bsum += b;
			npix++;
		}
	}

	Rsum /= (double)npix;
	Gsum /= (double)npix;
	Bsum /= (double)npix;

	borderColor = cv::Scalar((uchar)Bsum, (uchar)Gsum, (uchar)Rsum);

	return true;
}

std::string GetDateTimeString()
{
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	std::stringstream ss;

	ss << now->tm_mday << '-'
		<< (now->tm_mon + 1) << '-'
		<< (now->tm_year + 1900) << '-'
		<< now->tm_hour << '-'
		<< now->tm_min << '-'
		<< now->tm_sec;
	return ss.str();
}

bool CMapGameController::UpdateScores()
{
	int ActualMap = referenceMapHandler.GetActualRef();

	bool isHighScore = false;
	int currentHScore = 0;

	std::string HscoreFname;
	if (!scoreTrackers[ActualMap].getHighScore(currentHScore, HscoreFname))
	{
		isHighScore = true;
	}
	else if (Score > currentHScore)
	{
		isHighScore = true;
	}


	std::string fname = "";

	// Only dump hi-score images
	if (isHighScore)
	{
		fname = DataBaseDir + "Scores/" + referenceMapHandler.ReferenceMaps[ActualMap] + "_HiScoreMap_" + GetDateTimeString() + ".png";

		ofLogVerbose("AcquireScoreImage") << "writing score image ";

		ofImage tempImage;
		ofPixels tempPixels;
		fboProjWindow.readToPixels(tempPixels);

		tempImage.setFromPixels(tempPixels);

		ofSaveImage(tempImage, fname);

		ofLogVerbose("AcquireScoreImage") << "wrote score image ";
	}
	
	scoreTrackers[ActualMap].AddScore(Score, fname);
	scoreTrackers[ActualMap].SaveScoresXML(scoreFileNames[ActualMap]);
	ofLogVerbose("UpdateScores") << "Updated scores for " + referenceMapHandler.ReferenceMaps[ActualMap];
	return true;
}

bool CMapGameController::CreateScoreTexts(bool intermediate)
{
	int actRef = referenceMapHandler.GetActualRef();
	//std::string RefMap = referenceMapHandler.ReferenceMaps[actRef];
	std::string RefMapName = referenceMapHandler.ReferenceNames[actRef];

	std::stringstream ttext;
	int tscore = (int)Score;
	if (intermediate)
		ttext << RefMapName + " Current Score: " << tscore;
	else
		ttext << RefMapName + " Final Score: " << tscore;
	ScoreText = ttext.str();

	HiScoreText = "";

	bool PrintHScore = true;
	if (intermediate)
		PrintHScore = false;
	if (PrintHScore)
	{
		bool isHighScore = false;
		int currentHScore = 0;

		std::string HscoreFname;
		if (!scoreTrackers[actRef].getHighScore(currentHScore, HscoreFname))
		{
			isHighScore = true;
		}
		else if (Score > currentHScore)
		{
			isHighScore = true;
		}
		if (Score == 0)
			isHighScore = false;

		std::stringstream HScoreText;

		if (isHighScore)
		{
			HScoreText << "New High-Score! Great Job!";
		}
		else
		{
			int currentHScore;
			std::string fname;
			scoreTrackers[actRef].getHighScore(currentHScore, fname);
			HScoreText << RefMapName + " High-Score: " << currentHScore;
		}
		HiScoreText = HScoreText.str();
	}
	return true;
}


bool CMapGameController::ComputeProjectorMap()
{
	int nLMS = LMDepthImage.size();
				  
	// landmarks in projector coordinates
	std::vector<cv::Point2f> LMsProj(nLMS);
	for (int i = 0; i < nLMS; i++)
	{
		float x = LMDepthImage[i].x;
		float y = LMDepthImage[i].y;

		ofVec2f tp = kinectProjector->kinectCoordToProjCoord(x, y);
		LMsProj[i].x = tp.x;
		LMsProj[i].y = tp.y;
	}


	int actRef = referenceMapHandler.GetActualRef();
	std::string RefMap = referenceMapHandler.ReferenceMaps[actRef];
	std::string RefMapName = referenceMapHandler.ReferenceNames[actRef];

	std::string referenceImage = DataBaseDir + "ReferenceData/" + RefMap + ".png";

	ofImage temp;
	if (!temp.loadImage(referenceImage))
	{
		ofLogVerbose("CMapGameController") << "ComputeProjectorMap(): could not read " + referenceImage;
		return false;
	}
	cv::Mat RefImage = ofxCv::toCv(temp.getPixels());

	cv::Mat finalTrans = cv::getPerspectiveTransform(LMReferenceImage, LMsProj);

	int pw = projRes.x;
	int ph = projRes.y;
	cv::Mat IFinalOut = cv::Mat::zeros(ph, pw, RefImage.type());

	cv::Scalar borderColor(0, 0, 0);
	EstimateBorderColor(RefImage, borderColor);

	warpPerspective(RefImage, IFinalOut, finalTrans, IFinalOut.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, borderColor);

	unsigned char *ImgDat = (unsigned char*)(IFinalOut.data);
	matchResultImage.setFromPixels(ImgDat, IFinalOut.cols, IFinalOut.rows, OF_IMAGE_COLOR);
	//ShowScore = true;
	//LastTimeEvent = GetTimeStamp();


	return true;
}


void CMapGameController::DrawMatchResultContourLines()
{
	ofSetColor(255, 0, 0);
	//for (int i = 0; i < MatchResultContours.size(); i++)
	//{
	//	for (int j = 0; j < MatchResultContours[i].size() - 1; j++)
	//	{
	//		ofVec2f p1 = MatchResultContours[i][j];
	//		ofVec2f p2 = MatchResultContours[i][j + 1];

	//		ofSetLineWidth(5.0f);
	//		ofDrawLine(p1, p2);
	//	}
	//	ofDrawLine(MatchResultContours[i][0], MatchResultContours[i][MatchResultContours[i].size() - 1]);
	//}
	ofSetLineWidth(5.0f);
	for (int i = 0; i < MatchResultContours.size() - 1; i++)
	{
		ofVec2f p1 = MatchResultContours[i];
		ofVec2f p2 = MatchResultContours[i+1];

		ofDrawLine(p1, p2);
	}
	ofDrawLine(MatchResultContours[0], MatchResultContours[MatchResultContours.size() - 1]);
}
