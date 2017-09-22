/***********************************************************************
MapGameController.h - Controller for a Sandbox mapping / Island game
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


#ifndef _MapGameController_h_
#define _MapGameController_h_

#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ReferenceMapHandler.h"
#include "../KinectProjector/KinectProjector.h"
#include "SandboxScoreTracker.h"

//! Controller for the mapper game
/**  */
class CMapGameController
{
	public:
		//! Default constructor
		CMapGameController();

		//! Destructor
		virtual ~CMapGameController();

		void setup(std::shared_ptr<KinectProjector> const& k);
		void update();

		void PlayAndShowCountDown(int resultTime);

		void drawProjectorWindow();

		bool StartGame();

		// The button (or space) was pressed. Show map and subtract time
		bool ButtonPressed();

		// Signal to end the game
		bool EndButtonPressed();

		void TestMe();

		void RealTimeTestMe();
		void DebugTestMe();
				
		bool isIdle();
		
		void setProjectorRes(ofVec2f& PR);

		void setKinectRes(ofVec2f& KR);

		void setKinectROI(ofRectangle &KROI);

		ofRectangle getKinectROI();

		// Should debug files be dumped
		void setDebug(bool flag);

	private:
		
		std::shared_ptr<KinectProjector> kinectProjector;

		// Get the actual depth image
		bool GetActualBinaryLandImage();

		// Find largest connected component in binary image
		bool ConnectedComponentAnalysis();

		// Generate a set of Landmarks evenly spread over the template image
		void ComputeLandmarksFromTemplate(cv::Mat& I, std::vector<cv::Point2f> &LMs);
	
		// Returns true if the three points are almost collinear
		bool CheckCollinear(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);
	
		// Select four landmarks that are not collinear and are still within the ROI
		bool SelectFourGoodLms(std::vector<cv::Point2f> &LMs, std::vector<int> &goodIds);
	
		// Compute center of mas of binar image
		bool ComputeCM(const cv::Mat& I, double& CMx, double& CMy);
	
		// Translate reference image to center of mass
		bool ReferenceToCM(cv::Mat& I, cv::Mat& Iout, double &rCMx, double &rCMy);
	
		// Match a reference map to the largest connected component
		bool MatchMap();

		bool EstimateBorderColor(cv::Mat& RefImage, cv::Scalar& borderColor);

		bool UpdateScores();

		bool CreateScoreTexts(bool intermediate);
	
		bool ComputeProjectorMap();

		void DrawMatchResultContourLines();
	
		// Remove components connected to the border of the ROI
		void RemoveBorderBlobs(cv::Mat& image);
	
		// Read an image and cast into a binary image (0 background, 255 object)
		bool SafeReadBinaryImage(cv::Mat &img, const std::string &fname);

		// Compute the DICE score between two binary images.  
		void CompareImages(cv::Mat& I, cv::Mat& T, int Xi, int Yi, cv::Mat& MatchImage, double &DSC);

		void DrawScoresOnFBO();

		void DrawScoreTexts();

		// Since the OpenCV write functions does not work this hacky function is needed
		bool WriteGreyOpenCVMat(cv::Mat &img, const std::string &fname);
		bool WriteRGBOpenCVMat(cv::Mat &img, const std::string &fname);
		cv::Rect ROI;

		std::string DataBaseDir;
		CReferenceMapHandler referenceMapHandler;

		ofFbo fboProjWindow;

		ofImage matchResultImage;
		

		//std::vector<std::vector<ofVec2f> > MatchResultContours;
		std::vector<ofVec2f> MatchResultContours;

		bool doShowMatchResultContourLines;

		// Projector and kinect variables
		ofVec2f projRes;
		ofVec2f kinectRes;
		ofRectangle kinectROI;

		// Binary version of land image
		ofxCvGrayscaleImage BinaryLandImage;

		// Max BLOB image
		cv::Mat maxBLOBImage;

		// Landmarks on depth image
		std::vector<cv::Point2f> LMDepthImage;

		// Landmarks on reference image
		std::vector<cv::Point2f> LMReferenceImage;

		double MinSearchAngle;
		double MaxSearchAngle;

		double Score;

		ofTrueTypeFont scoreFont;
		ofTrueTypeFont nameFont;
		std::string ScoreText;
		std::string HiScoreText;

		// One for each map
		std::vector<CSandboxScoreTracker> scoreTrackers;
		std::vector<std::string> scoreFileNames;

		// State variables
		bool ShowScore;
		float LastTimeEvent;
		float ButtonPressTime;

		enum eGameState
		{
			GAME_STATE_IDLE,
			GAME_STATE_SHOWSPLASHSCREEN,
			GAME_STATE_CHECKFORISLAND,
			GAME_STATE_SHOWMAPNAME,
			GAME_STATE_PLAYANDSHOWCOUNTDOWN,
			GAME_STATE_SHOWINTERMIDEATERESULT,
			GAME_STATE_SHOWFINALRESULT
		};

		// Do the action required at the start of a game sequence
		bool InitiateGameSequence();
		bool CreateIntermediateResult();
		bool CheckForIsland();
		bool CreateFinalResult();
		int CheckedForIsland;
			
		bool CreateWelcomeScreen();
		bool CreateSplashScreen();
		
		ofImage splashScreen;

		// Drawable area
		ofRectangle projROI;

		std::vector<eGameState> GameSequence;
		std::vector<int> GameSequenceTimings;
		int CurrentGameSequence;

		void SetupGameSequence();
		bool ComputeWhereInSequence(int &currentStep, int &totalSteps);

		bool debugOn;
		std::string debugBaseDir;
};

#endif
