/***********************************************************************
ColorMap - ColorMap takes care of the colorMaps.
Copyright (c) 2016 Thomas Wolf
--- Adapted from Oliver Kreylos Vrui GLColorMap:
Copyright (c) 1999-2012 Oliver Kreylos

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

#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"

class ColorMap
{
public:
    struct HeightMapKey
    {
        float height;
        ofColor color;
        
        HeightMapKey(float h, ofColor c) : height(h), color(c) {}
        
        bool operator < (const HeightMapKey& hmk) const
        {
            return (height < hmk.height);
        }
    };
    
    ColorMap(void)
    :numEntries(512){
    }
    
    bool setKeys(std::vector<ofColor> colorkeys, std::vector<double> heightkeys); // Set keys
    bool updateColormap(void);    // Update colormap based on stored colorkeys
    bool setColorKey(int key, ofColor color);
    bool setHeightKey(int key, float height);
    bool addKey(ofColor color, float height);
    bool removeKey(int key);
    bool swapKeys(int k1, int k2);
    bool loadFile(string path);
    void saveFile(string filename);
    bool createFile(string filename); //create a sample colormap file
    HeightMapKey operator[](int scalar) const; // Return a key
    int size() const;
    ofTexture getTexture(); // return color map texture

    // Utilities
    bool scaleRange(float factor); // Rescale the range
    float getScalarRangeMin(void) const // Returns minimum of scalar value range
    {
        return min;
    }
    float getScalarRangeMax(void) const // Returns maximum of scalar value range
    {
        return max;
    }
    int getNumEntries(void) const // Returns the number of entries in the map
    {
        return numEntries;
    }
    int getNumKeys(void) const // Returns the number of colorkeys in the map
    {
        return heightMapKeys.size();
    }
    std::vector<HeightMapKey> getKeys(void) const // Returns the keys in the colormap
    {
        return heightMapKeys;
    }
    
private:
    // Colorkeys
    std::vector<HeightMapKey> heightMapKeys;
    
    //Colormap entries
    int numEntries; // Number of colors in the map
    ofPixels entries; // Array of RGBA entries
    ofImage tex;
    double min, max; // The scalar value range
};
