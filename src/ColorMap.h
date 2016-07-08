/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Inspired by Oliver Kreylos Vrui Colormap file.
 ***********************************************************************/

#pragma once

#include "ofMain.h"
//#include "ofxCv.h"
//#include "ofxOpenCv.h"
#include "ofxXmlSettings.h"

//using namespace ofxCv;
//using namespace cv;
class ColorMap
{
    /* Embedded classes: */
public:
    enum CreationTypes // Types for automatic palette generation
    {
        GREYSCALE=0x1,RAINBOW=0x2,
        CONSTANT_ALPHA=0x4,RAMP_ALPHA=0x8
    };

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
    
    ColorMap(void);
    ~ColorMap(void);

    /* Methods: */
    bool setKeys(std::vector<ofColor> colorkeys, std::vector<double> heightkeys); // Set keys
    bool updateColormap(void);    // Update colormap based on stored colorkeys
    void changeNumEntries(int amount, bool increase); // Changes the color map's size
    bool setColorKey(int key, ofColor color);
    bool setHeightKey(int key, float height);
    bool addKey(ofColor color, float height);
    bool loadFile(string path);
    void saveFile(string filename);
    bool createFile(string filename); //create a sample colormap file
    ofColor operator()(int scalar) const; // Return the color for a scalar value using linear interpolation
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
    double min,max; // The scalar value range
    //    double factor,offset; // The scaling factors to map data values to indices
    
    /* Private methods: */
    void copyMap(int newNumEntries,const ofColor* newEntries,double newMin,double newMax); // Copies from another color map
    
    /* Constructors and destructors: */
};
