/***********************************************************************
ReferenceMapHandler.h - Part of the Sandbox mapping / Island game
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


#ifndef _ReferenceMapHandler_h_
#define _ReferenceMapHandler_h_

#include <vector>
#include <string>

//! Handles maps for the map game
/**  */
class CReferenceMapHandler
{
	public:
		CReferenceMapHandler();

		virtual ~CReferenceMapHandler();

		void Init();

		int GetActualRef();

		void CycleMap();

		bool WriteToFile();
	
		bool ReadFromFile();
			
		// CycleMode: 0: no cycling, 1: follow map order, 2: random permutation
		void SetCycleMode(int mode);

		std::vector<std::string> ReferenceNames;
		std::vector<std::string> ReferenceMaps;


	private:
		void PermuteMapOrder();

		int DefaultMap;

		int ActualMap;

		// CycleMode: 0: no cycling, 1: follow map order, 2: random permutation
		int CycleMode;

		// For map cycling
		std::vector<int> IDList;
};

#endif
