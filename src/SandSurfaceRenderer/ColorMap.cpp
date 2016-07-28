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

#include "ColorMap.h"

bool ColorMap::updateColormap() {
    if (entries.isAllocated())
        entries.clear();
    entries.allocate(numEntries, 1, 3);

    /* Evaluate the color function: */
    for(int i=0;i<numEntries;++i)
    {
        /* Calculate the key value for this color map entry: */
        double val=double(i)*(heightMapKeys.back().height-heightMapKeys.front().height)/double(numEntries-1)+heightMapKeys.front().height;

        /* Find the piecewise linear segment of the color function containing the key value using binary search: */
        int l = 0;
        int r = heightMapKeys.size()-1;
		//if (i == 511)
		//	cout << "pof" << endl;
        while(r-l>1)
        {
            /* Enforce the invariant keys[l]<=val<keys[r]: */
            int m = (l+r)>>1;
            if(heightMapKeys[m].height<=val)
                l=m;
            else
                r=m;
        }
		//if (r == 17)
		//	cout << "paf" << endl;

        /* Evaluate the linear segment: */
        if(r<numEntries)
        {
            /* Interpolate linearly: */
            float w=float((val-heightMapKeys[l].height)/(heightMapKeys[r].height-heightMapKeys[l].height));
            ofColor tempcol = heightMapKeys[l].color*(1.0f-w)+heightMapKeys[r].color*w;
            entries.setColor(i,0,tempcol);
        }
        else
        {
            /* There is nothing to the right of the last key, so no need to interpolate: */
            entries.setColor(i,0,heightMapKeys.back().color);
        }
    }
    tex.setFromPixels(entries);
    return true;
}

bool ColorMap::scaleRange(float factor)
{
    for (auto k : heightMapKeys)
        k.height *= factor;
    min *= factor;
    max *= factor;
    return true;
}

bool ColorMap::setColorKey(int key, ofColor color){
    heightMapKeys[key].color = color;
    return updateColormap();
}

bool ColorMap::setHeightKey(int key, float height){
    heightMapKeys[key].height = height;
    
    std::sort(heightMapKeys.begin(), heightMapKeys.end());
    
    min = heightMapKeys.front().height;
    max = heightMapKeys.back().height;
    return updateColormap();
}

bool ColorMap::addKey(ofColor color, float height){
    heightMapKeys.push_back(HeightMapKey(height,color));
    
    std::sort(heightMapKeys.begin(), heightMapKeys.end());
    
    min = heightMapKeys.front().height;
    max = heightMapKeys.back().height;
    return updateColormap();
}

bool ColorMap::removeKey(int key){
    heightMapKeys.erase(heightMapKeys.begin()+key);
    min = heightMapKeys.front().height;
    max = heightMapKeys.back().height;
    return updateColormap();
}

bool ColorMap::swapKeys(int k1, int k2){
    ofColor tmp = heightMapKeys[k1].color;
    heightMapKeys[k1].color = heightMapKeys[k2].color;
    heightMapKeys[k2].color = tmp;
    return updateColormap();
}

ColorMap::HeightMapKey ColorMap::operator[](int scalar) const
{
    return heightMapKeys[scalar];
}

ofTexture ColorMap::getTexture(void)  // return color map texture
{
    return tex.getTexture();
}

int ColorMap::size() const
{
    return heightMapKeys.size();
}

bool ColorMap::loadFile(string filename) {
    heightMapKeys.clear();
    ofxXmlSettings settings;
    if(settings.loadFile(filename)){
        settings.pushTag("keys");
        int numberOfSavedPoints = settings.getNumTags("key");
        for(int i = 0; i < numberOfSavedPoints; i++){
            settings.pushTag("key", i);
            
            ofPoint p;
            float h = settings.getValue("height", 0);
            char r = settings.getValue("color-r", 0);
            char g = settings.getValue("color-g", 0);
            char b = settings.getValue("color-b", 0);
            ofColor c = ofColor(r, g, b);
            heightMapKeys.push_back(HeightMapKey(h, c));
            settings.popTag();
        }
        
        settings.popTag(); //pop position
    }
    else{
        ofLogError("HightMapKey file did not load!");
        return false;
    }

    min = heightMapKeys.front().height;
    max = heightMapKeys.back().height;
    
    return updateColormap();
}

void ColorMap::saveFile(string filename) {
    ofxXmlSettings positions;
    positions.addTag("keys");
    positions.pushTag("keys");
    //points is a vector<ofPoint> that we want to save to a file
    for(int i = 0; i < heightMapKeys.size(); i++){
        //each position tag represents one point
        positions.addTag("key");
        positions.pushTag("key",i);
        //so set the three values in the file
        positions.addValue("height", heightMapKeys[i].height);
        positions.addValue("color-r", heightMapKeys[i].color.r);
        positions.addValue("color-g", heightMapKeys[i].color.g);
        positions.addValue("color-b", heightMapKeys[i].color.b);
        positions.popTag();//pop position
    }
    positions.popTag(); //pop position
    positions.saveFile(filename);
}

bool ColorMap::createFile(string filename) {
    heightMapKeys.clear();
    
    int numKeys;
    {
        heightMapKeys.push_back( HeightMapKey(-220.0,ofColor(0, 0,  0)));
        heightMapKeys.push_back( HeightMapKey(-200.0,ofColor(0, 0,  80)));
        heightMapKeys.push_back( HeightMapKey(-170.0,ofColor(0,  30, 100)));
        heightMapKeys.push_back( HeightMapKey(-150.0,ofColor(0,  50, 102)));
        heightMapKeys.push_back( HeightMapKey(-125,ofColor(19, 108, 160)));
        heightMapKeys.push_back( HeightMapKey(-7.5,ofColor(24, 140, 205)));
        heightMapKeys.push_back( HeightMapKey(-2.5,ofColor(135, 206, 250)));
        heightMapKeys.push_back( HeightMapKey(-0.5,ofColor(176, 226, 255)));
        heightMapKeys.push_back( HeightMapKey(0.0,ofColor(0,  97,  71)));
        heightMapKeys.push_back( HeightMapKey(2.5,ofColor(16, 122,  47)));
        heightMapKeys.push_back( HeightMapKey(25,ofColor(232, 215, 125)));
        heightMapKeys.push_back( HeightMapKey(60,ofColor(161,  67,   0)));
        heightMapKeys.push_back( HeightMapKey(90,ofColor(130,  30,  30)));
        heightMapKeys.push_back( HeightMapKey(140,ofColor(161, 161, 161)));
        heightMapKeys.push_back( HeightMapKey(200,ofColor(206, 206, 206)));
        heightMapKeys.push_back( HeightMapKey(220,ofColor(255, 255, 255)));
    }

    ofxXmlSettings positions;
    positions.addTag("keys");
    positions.pushTag("keys");
    //points is a vector<ofPoint> that we want to save to a file
    for(int i = 0; i < heightMapKeys.size(); i++){
        //each position tag represents one point
        positions.addTag("key");
        positions.pushTag("key",i);
        //so set the three values in the file
        positions.addValue("height", heightMapKeys[i].height);
        positions.addValue("color-r", heightMapKeys[i].color.r);
        positions.addValue("color-g", heightMapKeys[i].color.g);
        positions.addValue("color-b", heightMapKeys[i].color.b);
        positions.popTag();//pop position
    }
    positions.popTag(); //pop position
    positions.saveFile(filename);
    
    min = heightMapKeys.front().height;
    max = heightMapKeys.back().height;
    
    return updateColormap();
}
