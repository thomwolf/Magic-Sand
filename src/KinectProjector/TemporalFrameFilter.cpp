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


#include "TemporalFrameFilter.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "ofLog.h"

CTemporalFrameFilter::CTemporalFrameFilter()
{
	medianImg = nullptr;
	imgDataBuffer = nullptr;
	imgDataBufferCol = nullptr;
	currentFrame = 0;
	validBuffer = false;
	sizeX = 0;
	sizeY = 0;
	nFrames = 0;
}

CTemporalFrameFilter::~CTemporalFrameFilter()
{
	ClearData();
}

void CTemporalFrameFilter::Init(int sx, int sy, int frames)
{
	ClearData();
	sizeX = sx;
	sizeY = sy;
	nFrames = frames;
	imgDataBuffer = new unsigned char[sx * sy * frames];
	medianImg = new unsigned char[sx * sy];
	imgDataBufferCol = new unsigned char[sx * sy * 3 * frames];
	validBuffer = false;
	currentFrame = 0;
}

void CTemporalFrameFilter::NewFrame(unsigned char* imgData, int sx, int sy, int nFrames)
{
	if (!imgDataBuffer)
	{
		ofLogVerbose("CTemporalFrameFilter") << "NewFrame(): No buffer allocated: allocating";
		Init(sx, sy, nFrames);
	}
	int offset = currentFrame * sizeX * sizeY;
	for (int i = 0; i < sizeX *sizeY; i++)
	{
		unsigned char R = imgData[3*i];
		unsigned char G = imgData[3 * i+1];
		unsigned char B = imgData[3 * i+2];
		double IV = (R + G + B) / 3;
		imgDataBuffer[offset + i] = (unsigned char)IV;
	}

	currentFrame++;
	if (currentFrame >= nFrames)
	{
		validBuffer = true;
		currentFrame = 0;
	}
}


void CTemporalFrameFilter::NewColFrame(unsigned char* imgData, int sx, int sy, int nFrames /*= 15*/)
{
	if (!imgDataBufferCol)
	{
		std::cerr << "CTemporalFrameFilter::NewFrame: No color buffer allocated: allocating" << std::endl;
		Init(sx, sy, nFrames);
	}
	int offset = currentFrame * sizeX * sizeY * 3;
	for (int i = 0; i < sizeX *sizeY; i++)
	{
		unsigned char R = imgData[3 * i];
		unsigned char G = imgData[3 * i + 1];
		unsigned char B = imgData[3 * i + 2];
		imgDataBufferCol[offset + 3 * i + 0] = R;
		imgDataBufferCol[offset + 3 * i + 1] = G;
		imgDataBufferCol[offset + 3 * i + 2] = B;
	}
	currentFrame++;
	if (currentFrame >= nFrames)
	{
		validBuffer = true;
		currentFrame = 0;
	}

}

int CTemporalFrameFilter::getBufferSize()
{
	return nFrames;
}

bool CTemporalFrameFilter::isValid()
{
	return validBuffer;
}

void CTemporalFrameFilter::ClearData()
{
	if (imgDataBuffer)
	{
		delete[] imgDataBuffer;
		imgDataBuffer = nullptr;
	}
	if (medianImg)
	{
		delete[] medianImg;
		medianImg = nullptr;
	}
	if (imgDataBufferCol)
	{
		delete[] imgDataBufferCol;
		imgDataBufferCol = nullptr;
	}

	currentFrame = 0;
	validBuffer = false;
}

// https://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation
template <typename T = double, typename C>
inline const T median(const C &the_container)
{
	std::vector<T> tmp_array(std::begin(the_container),
		std::end(the_container));
	size_t n = tmp_array.size() / 2;
	std::nth_element(tmp_array.begin(), tmp_array.begin() + n, tmp_array.end());

	if (tmp_array.size() % 2) { return tmp_array[n]; }
	else
	{
		// even sized vector -> average the two middle values
		auto max_it = std::max_element(tmp_array.begin(), tmp_array.begin() + n);
		return (*max_it + tmp_array[n]) / 2.0;
	}
}

unsigned char* CTemporalFrameFilter::getMedianFilteredImage()
{
	if (!ComputeMedianImage())
		return nullptr;

	return medianImg;
}

unsigned char* CTemporalFrameFilter::getAverageFilteredColImage()
{
	if (!ComputeAverageImageCol())
		return nullptr;

	return medianImg;

}

bool CTemporalFrameFilter::ComputeMedianImage()
{
	if (!validBuffer)
		return false;

	std::vector<unsigned char> tvals(nFrames);
	
	for (int y = 0; y < sizeY; y++)
	{
		for (int x = 0; x < sizeX; x++)
		{
			for (int f = 0; f < nFrames; f++)
			{
				int offset = f * sizeX * sizeY;
				int idx = offset + y*sizeX + x;
				tvals[f] = imgDataBuffer[idx];
			}
			unsigned char med = (unsigned char)median(tvals);
			int idx2 = y*sizeX + x;	
			medianImg[idx2] = med;
		}
	}

	return true;
}

bool CTemporalFrameFilter::ComputeAverageImageCol()
{
	if (!validBuffer && nFrames > 0)
		return false;

	for (int y = 0; y < sizeY; y++)
	{
		for (int x = 0; x < sizeX; x++)
		{
			double RSum = 0;
			double GSum = 0;
			double BSum = 0;
			for (int f = 0; f < nFrames; f++)
			{
				int offset = f * sizeX * sizeY * 3;
				int idx = offset + (y*sizeX + x) * 3;
				unsigned char R = imgDataBufferCol[idx + 0];
				unsigned char G = imgDataBufferCol[idx + 1];
				unsigned char B = imgDataBufferCol[idx + 2];

				RSum += R;				
				GSum += G;				
				BSum += B;
			}
			RSum /= nFrames;
			GSum /= nFrames;
			BSum /= nFrames;

			unsigned char val = (unsigned char)((RSum + GSum + BSum) / 3.0);
			int idx2 = y*sizeX + x;
			medianImg[idx2] = val;
		}
	}

	return true;
}
