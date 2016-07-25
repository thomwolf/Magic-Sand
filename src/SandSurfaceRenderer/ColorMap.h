/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Adapted from Oliver Kreylos Vrui Colormap file:
 https://github.com/KeckCAVES/Vrui
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
