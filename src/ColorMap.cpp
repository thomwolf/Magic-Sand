/***********************************************************************
 ColorMap - Class to map from scalar values to RGBA colors.
 Inspired by Oliver Kreylos Vrui Colormap file
 which is part of the OpenGL Support Library (GLSupport).
 ***********************************************************************/

#include <ColorMap.h>
using namespace ofxCv;
using namespace cv;

ColorMap::~ColorMap(void)
{
}

void ColorMap::setNumEntries(int newNumEntries)
{
    /* Check if number actually changed: */
    if(numEntries!=newNumEntries)
    {
        /* Reallocate entry array: */
        numEntries=newNumEntries;

        /* Recalculate mapping factors: */
        factor=double(numEntries-1)/(max-min);
        offset=min*factor;
    }
}

bool ColorMap::load(string filename, bool absolute) {
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::READ);
    FileNode features = fs["ColorMap"];
    FileNodeIterator it = features.begin(), it_end = features.end();
    int idx = 0;
    std::vector<uchar> lbpval;
    
    // iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it, idx++ )
    {
        cout << "color #" << idx << ": ";
        cout << "z=" << (double)(*it)["z"] << ", color=" << (int)(*it)["color"] << endl;
        heightMapKeys.push_back((double)(*it)["z"]);
        heightMapColors.push_back(ofColor::fromHex((int)(*it)["color"]));
    }
    fs.release();
    return updateColormap();
}

bool ColorMap::setKeys(std::vector<ofColor> colorkeys, std::vector<double> heightkeys) {
    heightMapKeys = heightkeys;
    heightMapColors = colorkeys;
    return updateColormap();
}

bool ColorMap::updateColormap() {
    /* Create entry array: */
    numKeys = heightMapKeys.size();
    numEntries = 256;
    
    setScalarRange(heightMapKeys[0],heightMapKeys[numKeys-1]);

    if (entries.isAllocated())
        entries.clear();
    entries.allocate(numEntries, 1, 3);

    /* Evaluate the color function: */
    for(int i=0;i<numEntries;++i)
    {
        /* Calculate the key value for this color map entry: */
        double val=double(i)*(heightMapKeys[numKeys-1]-heightMapKeys[0])/double(numEntries-1)+heightMapKeys[0];

        /* Find the piecewise linear segment of the color function containing the key value using binary search: */
        int l=0;
        int r=numKeys;
        while(r-l>1)
        {
            /* Enforce the invariant keys[l]<=val<keys[r]: */
            int m=(l+r)>>1;
            if(heightMapKeys[m]<=val)
                l=m;
            else
                r=m;
        }

        /* Evaluate the linear segment: */
        if(r<numEntries)
        {
            /* Interpolate linearly: */
            float w=float((val-heightMapKeys[l])/(heightMapKeys[r]-heightMapKeys[l]));
            ofColor tempcol = heightMapColors[l]*(1.0f-w)+heightMapColors[r]*w;
            entries.setColor(i,0,tempcol);
        }
        else
        {
            /* There is nothing to the right of the last key, so no need to interpolate: */
            entries.setColor(i,0,heightMapColors[numKeys-1]);
        }
    }
    tex.setFromPixels(entries);
    return true;
}

bool ColorMap::setScalarRange(double newMin,double newMax)
{
    min=newMin;
    max=newMax;
    factor=double(numEntries-1)/(max-min);
    offset=min*factor;

    return true;
}

bool ColorMap::createFile(string filename, bool absolute) {
    std::vector<ofColor> heightMapColors;
    std::vector<double> heightMapKeys;
    int numKeys;
    {
    heightMapColors.push_back( ofColor(0, 0,  80));
    heightMapColors.push_back( ofColor(0,  30, 100));
    heightMapColors.push_back( ofColor(0,  50, 102));
    heightMapColors.push_back( ofColor(19, 108, 160));
    heightMapColors.push_back( ofColor(24, 140, 205));
    heightMapColors.push_back( ofColor(135, 206, 250));
    heightMapColors.push_back( ofColor(176, 226, 255));
    heightMapColors.push_back( ofColor(0,  97,  71));
    heightMapColors.push_back( ofColor(16, 122,  47));
    heightMapColors.push_back( ofColor(232, 215, 125));
    heightMapColors.push_back( ofColor(161,  67,   0));
    heightMapColors.push_back( ofColor(130,  30,  30));
    heightMapColors.push_back( ofColor(161, 161, 161));
    heightMapColors.push_back( ofColor(206, 206, 206));
    heightMapColors.push_back( ofColor(255, 255, 255));
    heightMapKeys.push_back(-40.0);
    heightMapKeys.push_back(-30.0);
    heightMapKeys.push_back(-20.0);
    heightMapKeys.push_back(-12.5);
    heightMapKeys.push_back(-0.75);
    heightMapKeys.push_back(-0.25);
    heightMapKeys.push_back(-0.05);
    heightMapKeys.push_back(0.0);
    heightMapKeys.push_back(0.25);
    heightMapKeys.push_back(2.5);
    heightMapKeys.push_back(6);
    heightMapKeys.push_back(9);
    heightMapKeys.push_back(14);
    heightMapKeys.push_back(20);
    heightMapKeys.push_back(25);
    }
    FileStorage fs(ofToDataPath(filename, absolute), FileStorage::WRITE);
    fs << "ColorMap" << "[";
    for( int i = 0; i < heightMapColors.size(); i++ )
    {
        fs << "{:" << "z" << heightMapKeys[i] << "color" << heightMapColors[i].getHex() << "}";
    }
    fs << "]";
    return true;
}

ColorMap::Color ColorMap::operator()(int scalar) const
{
    ofColor color = entries.getColor(scalar, 0);
    return color;
}

ofTexture ColorMap::getTexture(void)  // return color map
{
    return tex.getTexture();
}

