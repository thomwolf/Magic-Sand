/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Inspired by Oliver Kreylos Vrui Colormap file
 which is part of the OpenGL Support Library (GLSupport).
 ***********************************************************************/

#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"

using namespace ofxCv;
using namespace cv;
class ColorMap
{
    /* Embedded classes: */
public:
    typedef ofColor Color; // Type of color entries

    enum CreationTypes // Types for automatic palette generation
    {
        GREYSCALE=0x1,RAINBOW=0x2,
        CONSTANT_ALPHA=0x4,RAMP_ALPHA=0x8
    };

    /* Elements: */
private:
    // Colorkeys
    int numKeys;
    std::vector<ofColor> heightMapColors; // Color keys
    std::vector<double> heightMapKeys;  // Height keys

    //Colormap entries
    int numEntries; // Number of colors in the map
    ofPixels entries; // Array of RGBA entries
    ofImage tex;
    double min,max; // The scalar value range
    double factor,offset; // The scaling factors to map data values to indices

    /* Private methods: */
    void setNumEntries(int newNumEntries); // Changes the color map's size
    void copyMap(int newNumEntries,const Color* newEntries,double newMin,double newMax); // Copies from another color map

    /* Constructors and destructors: */
public:
    ~ColorMap(void);

    /* Methods: */
    bool load(string path, bool absolute = false); // Loads colorkeys from a file
    bool setKeys(std::vector<ofColor> colorkeys, std::vector<double> heightkeys); // Set keys
    bool updateColormap(void);    // Update colormap based on stored colorkeys

    bool createFile(string filename, bool absolute); //create a sample colormap file
    
    Color operator()(int scalar) const; // Return the color for a scalar value using linear interpolation
    ofTexture getTexture(); // return color map texture

    // Utilities
    bool setScalarRange(double newMin,double newMax);
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
        return numKeys;
    }
    std::vector<ofColor> getColorKeys(void) const // Returns the colorkeys in the map
    {
        return heightMapColors;
    }
    std::vector<double> getHeightKeys(void) const // Returns the heightkeys in the map
    {
        return heightMapKeys;
    }
    
};
