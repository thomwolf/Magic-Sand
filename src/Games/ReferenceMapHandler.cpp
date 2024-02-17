/***********************************************************************
ReferenceMapHandler.cpp - Part of the Sandbox mapping / Island game
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


#include "ReferenceMapHandler.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include "ofMain.h"


CReferenceMapHandler::CReferenceMapHandler()
{
	DefaultMap = 0;
	ActualMap = DefaultMap;
	CycleMode = 0;
}


CReferenceMapHandler::~CReferenceMapHandler()
{
}


void CReferenceMapHandler::Init()
{
	ReadFromFile();
	std::cout << "read " << ReferenceMaps.size() << " reference maps" << std::endl;
}

int CReferenceMapHandler::GetActualRef()
{
	if (CycleMode == 0)
		return ActualMap;

	if (CycleMode == 1)
	{
		return ActualMap;
	}
	if (CycleMode == 2)
	{
		if (ActualMap >= IDList.size()-1)
		{
			return ActualMap;
		}
		else
		{
			return IDList[ActualMap];
		}
	}

	return ActualMap;
}

void CReferenceMapHandler::CycleMap()
{
	if (CycleMode == 0)
		return;
	if (CycleMode == 1)
	{
		ActualMap++;
		if (ActualMap >= ReferenceMaps.size())
			ActualMap = 0;
	}
	if (CycleMode == 2)
	{
		if (IDList.size() == 0)
		{
			PermuteMapOrder();
		}
		ActualMap++;
		if (ActualMap >= ReferenceMaps.size())
		{
			PermuteMapOrder();
		}
	}
}

void CReferenceMapHandler::PermuteMapOrder()
{
	ofSeedRandom();
	IDList.resize(ReferenceMaps.size());

	for (int i = 0; i < ReferenceMaps.size(); i++)
	{
		IDList[i] = i;
	}
	for (int i = 0; i < IDList.size(); i++)
	{
		int newID = (int)ofRandom(IDList.size() - 1);

		int t = IDList[i];
		IDList[i] = IDList[newID];
		IDList[newID] = t;
	}
}

bool CReferenceMapHandler::WriteToFile()
{
	std::string refName = "mapGame/ReferenceData/MapReferenceSettings.xml";

	ofXml XMLOut;
	auto mapRef = XMLOut.appendChild("MapReferenceSettings");
//	XMLOut.setTo("MapReferenceSettings");
    mapRef.appendChild("DefaultMap").set(0);

    auto maps = mapRef.appendChild("maps");

	for (int i = 0; i < ReferenceMaps.size(); i++)
	{
        auto map = maps.appendChild("map");
//		XMLOut.setTo("map["+ ofToString(i) +"]");
        map.setAttribute("id", ofToString(i));
        map.appendChild("MapName").set(ReferenceNames[i]);
        map.appendChild("GroundTruth").set(ReferenceMaps[i]);
	}

	return XMLOut.save(refName);
}


bool CReferenceMapHandler::ReadFromFile()
{
	std::string refName = "mapGame/ReferenceData/MapReferenceSettings.xml";

	ReferenceNames.clear();
	ReferenceMaps.clear();

	ofXml XMLIn;
	if (!XMLIn.load(refName))
	{
		std::cout << "Could not read " << refName << std::endl;
		return false;
	}

    auto mapRefSettings = XMLIn.getFirstChild();

	DefaultMap = mapRefSettings.getChild("DefaultMap").getIntValue();
	ActualMap = DefaultMap;

	auto mapsXML = mapRefSettings.find("map"); // how many do you have?

	for (auto & map : mapsXML)
	{
		std::string rn = map.getChild("MapName").getValue<string>();
		std::string rGT = map.getChild("GroundTruth").getValue<string>();

		ReferenceNames.push_back(rn);
		ReferenceMaps.push_back(rGT);
	}

	return true;
}

void CReferenceMapHandler::SetCycleMode(int mode)
{
	CycleMode = mode;

	if (CycleMode == 2)
	{
		PermuteMapOrder();
	}
}

