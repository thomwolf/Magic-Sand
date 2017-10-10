/***********************************************************************
TemporalFrameFilter.h - Takes care of temporal filtering of colour images
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
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef _TemporalFrameFilter_h_
#define _TemporalFrameFilter_h_

//! Temporal frame filter for colour images
/** Can do temporal average and temporal median filtering
    Can be used for dealing with rolling shutter effects etc.*/
class CTemporalFrameFilter
{
	public:
		CTemporalFrameFilter();

		virtual ~CTemporalFrameFilter();

		void Init(int sx, int sy, int frames);

		void NewFrame(unsigned char* imgData, int sx, int sy, int nFrames = 15);

		void NewColFrame(unsigned char* imgData, int sx, int sy, int nFrames = 50);

		int getBufferSize();

		bool isValid();

		unsigned char* getMedianFilteredImage();

		unsigned char* getAverageFilteredColImage();

	private:
		unsigned char *imgDataBuffer;

		unsigned char *medianImg;

		unsigned char *imgDataBufferCol;

		int currentFrame;

		bool validBuffer;

		void ClearData();
		
		bool ComputeMedianImage();

		bool ComputeAverageImageCol();

		int sizeX;

		int sizeY;

		int nFrames;

};

#endif
