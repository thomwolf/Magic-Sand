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

class SaveModal : public ofxModalWindow
{
public:
    SaveModal(std::shared_ptr<ofxModalTheme> theme){
        setTheme(theme);
        setTitle("Save Color Map File");
        textInput = addComponent(new ofxDatGuiTextInput("file name", ""));
        addButton("Cancel");
        ofxDatGuiButton* closeButton = getButton(0);
        closeButton->setLabel("Save");
        closeButton->setLabelColor(ofColor::fromHex(0xffffff));
        closeButton->setBackgroundColors(
                                         ofColor::fromHex(0x337ab7),     // normal //
                                         ofColor::fromHex(0x286090),     // on mouse over //
                                         ofColor::fromHex(0x1f4c73));      // on mouse down //
        closeButton->setBorder(ofColor::fromHex(0x1f4c73), 1);
        autoSize();
    }
    void show() {
        textInput->setFocused(true);
        ofxModalWindow::show();
    }
    string getTextInput(){
        return textInput->getText();
    }
private:
    ofxDatGuiTextInput* textInput;
};

class SandSurfaceRenderer {
public:
    SandSurfaceRenderer(std::shared_ptr<KinectProjector> const& k)
    :settingsLoaded(false) {
        kinectProjector = k;
    }
    ~SandSurfaceRenderer(void);
    
    void setup(ofVec2f sprojRes);
    void setupMesh();
    void updateConversionMatrices();
    void updateRangesAndBasePlane();
    void update();
    void drawMainWindow(float x, float y, float width, float height);
    void drawProjectorWindow();
    void drawSandbox();
    void prepareContourLinesFbo();
    
    void setupGui();
    void updateColorListColor(int i, int j);
    void populateColorList();
    void onButtonEvent(ofxDatGuiButtonEvent e);
    void onToggleEvent(ofxDatGuiToggleEvent e);
    void onSliderEvent(ofxDatGuiSliderEvent e);
    void onColorPickerEvent(ofxDatGuiColorPickerEvent e);
    void onDropdownEvent(ofxDatGuiDropdownEvent e);
    void onScrollViewEvent(ofxDatGuiScrollViewEvent e);
    void onSaveModalEvent(ofxModalEvent e);
    
    bool loadSettings();
    bool saveSettings();
private:
    std::shared_ptr<KinectProjector> kinectProjector;
    bool settingsLoaded;
    
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
    string colorMapPath;
    string colorMapFile;
    std::vector<string> colorMapFilesList;
    ColorMap    heightMap;
    std::vector<ColorMap::HeightMapKey> heightMapKeys;
    
	float heightMapScale,heightMapOffset; // Scale and offset values to convert from elevation to height color map texture coordinates
    float contourLineFboScale, contourLineFboOffset; // Scale and offset values to convert depth from contourline shader values to real values
	float FilteredDepthScale,FilteredDepthOffset; // Scale and offset values to convert depth from normalized shader values to real values
    float elevationMin, elevationMax;
    
    // Contourlines
    float contourLineDistance, contourLineFactor;
    bool drawContourLines; // Flag if topographic contour lines are enabled
    
    // GUI Main interface and Modal
    ofxDatGui* gui;
    ofxDatGuiScrollView* colorList;
    int selectedColor;
    shared_ptr<SaveModal> saveModal;
};
