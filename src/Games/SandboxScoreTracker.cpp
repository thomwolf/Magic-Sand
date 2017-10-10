/***********************************************************************
SandboxScoreTracker.h - Keep tracks of score for the Sandbox games
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

#include "SandboxScoreTracker.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include "ofMain.h"

CSandboxScoreTracker::CSandboxScoreTracker()
{
	wasHiScore = false;
}

CSandboxScoreTracker::~CSandboxScoreTracker()
{
}

std::string CSandboxScoreTracker::GetDateTimeString()
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


bool CSandboxScoreTracker::AddScore(int score, std::string &scoreImage)
{
	scores.push_back(score);
	scoreImages.push_back(scoreImage);
	std::string scDate = GetDateTimeString();
	scoreDates.push_back(scDate);

	// slow way to sort
	int p = scores.size() - 1;
	while (p > 0)
	{
		if (scores[p - 1] < score)
		{
			// Shuffle
			int ts = scores[p - 1];
			std::string tt = scoreImages[p - 1];
			std::string td = scoreDates[p - 1];
			scores[p - 1] = score;
			scoreImages[p - 1] = scoreImage;
			scoreDates[p - 1] = scDate;
			scores[p] = ts;
			scoreImages[p] = tt;
			scoreDates[p] = td;
		}
		p--;
	}
	if (scores[0] == score)
		wasHiScore = true;

	return wasHiScore;
}

bool CSandboxScoreTracker::WasHiScore()
{
	return wasHiScore;
}

int CSandboxScoreTracker::getNumberOfScore()
{
	return scores.size();
}

int CSandboxScoreTracker::getScore(int idx)
{
	return scores[idx];
}

std::string CSandboxScoreTracker::getScoreImage(int idx)
{
	return scoreImages[idx];
}

bool CSandboxScoreTracker::SaveScoresXML(std::string &fname)
{
	ofXml XMLOut;
	XMLOut.addChild("scores");
	XMLOut.setTo("scores");

	for (int i = 0; i < scores.size(); i++)
	{
		XMLOut.addChild("score");
		XMLOut.setTo("score[" + ofToString(i) + "]");
		XMLOut.setAttribute("id", ofToString(i));
		XMLOut.addValue("value", scores[i]);
		XMLOut.addValue("image", scoreImages[i]);
		XMLOut.addValue("date", scoreDates[i]);
		XMLOut.setToParent();
	}
	return XMLOut.save(fname);
}

bool CSandboxScoreTracker::LoadScoresXML(std::string &fname)
{
	ofXml XMLIn;
	if (!XMLIn.load(fname))
	{
		std::cout << "Could not read " << fname << std::endl;
		return false;
	}
	scores.clear();
	scoreImages.clear();
	scoreDates.clear();

	XMLIn.setTo("scores");

	int nscores = XMLIn.getNumChildren(); // how many do you have?

	for (int i = 0; i < nscores; i++)
	{
		if (XMLIn.setTo("score[" + ofToString(i) + "]"))
		{
			int tsc = XMLIn.getValue<int>("value");
			std::string tI = XMLIn.getValue<string>("image");
			std::string tD = XMLIn.getValue<string>("date");

			scores.push_back(tsc);
			scoreImages.push_back(tI);
			scoreDates.push_back(tD);

			XMLIn.setToParent();
		}
	}

	return true;
}

bool CSandboxScoreTracker::getHighScore(int &score, std::string &fname)
{
	if (scores.size() == 0)
		return false;

	score = scores[0];
	fname = scoreImages[0];
	return true;
}

void CSandboxScoreTracker::ResetHighScores(std::string fname)
{


	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	std::stringstream ss;

	ss << fname << "_Backup_"
		<< now->tm_mday << '-'
		<< (now->tm_mon + 1) << '-'
		<< (now->tm_year + 1900) << '-'
		<< now->tm_hour << '-'
		<< now->tm_min << '-'
		<< now->tm_sec
		<< ".txt";

	std::string orgname = fname + ".txt";

	std::rename(orgname.c_str(), ss.str().c_str());

	scoreImages.clear();
	scores.clear();
}

