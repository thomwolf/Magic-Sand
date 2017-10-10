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


#ifndef _SandboxScoreTracker_h_
#define _SandboxScoreTracker_h_

#include <vector>
#include <string>

//! Keeps tracks of scores and hi-score
/**  */
class CSandboxScoreTracker
{
	public:
		//! Default constructor
		CSandboxScoreTracker();

		//! Destructor
		virtual ~CSandboxScoreTracker();

		// Returns true if high-score
		bool AddScore(int score, std::string &scoreImage);

		// Was the last score added a hi-score?
		bool WasHiScore();

		int getNumberOfScore();

		int getScore(int idx);

		std::string getScoreImage(int idx);

		bool SaveScoresXML(std::string &fname);
		
		bool LoadScoresXML(std::string &fname);
		
		
		bool getHighScore(int &score, std::string &fname);

		void ResetHighScores(std::string fname);

	private:

		std::vector<int> scores;
		std::vector<std::string> scoreImages;
		std::vector<std::string> scoreDates;

		std::string GetDateTimeString();

		bool wasHiScore;
};

#endif
