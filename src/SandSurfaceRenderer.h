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
    
    void setup(ofVec2f sprojRes);
    void setupMesh();
    void updateConversionMatrices();
    void updateRangesAndBasePlane();
    void update();
    void drawMainWindow();
    void drawProjectorWindow();
    void drawSandbox();
    void prepareContourLinesFbo();
    
    void setupGui();
    void updateColorListColor(int i, int j);
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onColorPickerEvent(ofxDatGuiColorPickerEvent e);
    void onScrollViewEvent(ofxDatGuiScrollViewEvent e);
    
private:
    std::shared_ptr<KinectProjector> kinectProjector;
    
    // Projector Resolution
    int projResX, projResY;
    
    // Conversion matrices
    ofMatrix4x4                 transposedKinectProjMatrix;
    ofMatrix4x4                 transposedKinectWorldMatrix;

    // Mesh
    ofMesh mesh;
    int meshwidth;          //Mesh size
    int meshheight;
    
    // Shaders
    ofShader elevationShader;
    ofShader heightMapShader;
    
    // FBos
    ofFbo   fboProjWindow;    
    ofFbo   contourLineFramebufferObject;

    // Base plane
    ofVec3f basePlaneNormal, basePlaneNormalBack;
    ofVec3f basePlaneOffset, basePlaneOffsetBack;
    ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format

    // Colormap
    ColorMap    heightMap;
    std::vector<ColorMap::HeightMapKey> heightMapKeys;
    
	float heightMapScale,heightMapOffset; // Scale and offset values to convert from elevation to height color map texture coordinates
    float contourLineFboScale, contourLineFboOffset; // Scale and offset values to convert depth from contourline shader values to real values
	float FilteredDepthScale,FilteredDepthOffset; // Scale and offset values to convert depth from normalized shader values to real values
    
    // Contourlines
    float contourLineDistance, contourLineFactor;
    bool drawContourLines; // Flag if topographic contour lines are enabled
    
    float elevationMin, elevationMax;
    
    // GUI Main interface
    ofxDatGui* gui;
    ofxDatGuiScrollView* colorList;
    int selectedColor;
};
