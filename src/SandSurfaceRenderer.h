//
//  SandSurfaceRenderer.h
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#ifndef __GreatSand__SandSurfaceRenderer__
#define __GreatSand__SandSurfaceRenderer__

#include <iostream>
#include "ofMain.h"

#include "KinectProjector.h"
#include "ColorMap.h"
#include "Utils.h"
#endif /* defined(__GreatSand__SandSurfaceRenderer__) */

class SandSurfaceRenderer {
public:
    SandSurfaceRenderer(std::shared_ptr<KinectProjector> const& k) {
        kinectProjector = k;
    }
    
    void setup(int sprojResX, int sprojResY);
    void setupMesh();
    void setRangesAndBasePlaneEquation();
    void computeBasePlane();
  
    void update();
    
    void draw();
    void drawSandbox();
    void prepareContourLines();
private:
    std::shared_ptr<KinectProjector> kinectProjector;
    
    // Projector Resolution
    int projResX, projResY;
    
    // Mesh
    ofMesh mesh;
    int meshwidth;          //Mesh size
    int meshheight;
    
    // Shaders
    ofShader elevationShader;
    ofShader heightMapShader;
    
    // FBos
    ofFbo   contourLineFramebufferObject;

    // Base plane
    ofVec3f basePlaneNormal, basePlaneNormalBack;
    ofVec3f basePlaneOffset, basePlaneOffsetBack;
    ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format

    // Colormap, contourmap and heightmap variables
    ColorMap    heightMap;
	float heightMapScale,heightMapOffset; // Scale and offset values to convert from elevation to height color map texture coordinates
    float contourLineFboScale, contourLineFboOffset; // Scale and offset values to convert depth from contourline shader values to real values
	float FilteredDepthScale,FilteredDepthOffset; // Scale and offset values to convert depth from normalized shader values to real values
    
    // Contourlines
    float contourLineDistance, contourLineFactor;
    bool drawContourLines; // Flag if topographic contour lines are enabled
    
    float elevationMin, elevationMax;
};
