//
//  SandSurfaceRenderer.cpp
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#include "SandSurfaceRenderer.h"

using namespace ofxCSG;

void SandSurfaceRenderer::setup(ofVec2f sprojRes){
    // Sandbox contourlines
    drawContourLines = true; // Flag if topographic contour lines are enabled
	contourLineDistance = 10.0; // Elevation distance between adjacent topographic contour lines in millimiters
    
    // Initialize the fbos and images
    projResX = sprojRes.x;
    projResY = sprojRes.y;
    contourLineFramebufferObject.allocate(projResX+1, projResY+1, GL_RGBA);
    contourLineFramebufferObject.begin();
    ofClear(0,0,0,255);
    contourLineFramebufferObject.end();

    // Load colormap and set heightmap
    if (!heightMap.loadFile("HeightColorMap.xml"))
        heightMap.createFile("HeightColorMap.xml");
    
//    heightMap = heightMap.getKeys();
    
    //Set elevation Min and Max
    elevationMin = -heightMap.getScalarRangeMin();
    elevationMax = -heightMap.getScalarRangeMax();
    
    // Calculate the  height map elevation scaling and offset coefficients
	heightMapScale = (heightMap.getNumEntries()-1)/((elevationMax-elevationMin));
	heightMapOffset = 0.5/heightMap.getNumEntries()-heightMapScale*elevationMin;
    
    // Calculate the contourline fbo scaling and offset coefficients
	contourLineFboScale = elevationMin-elevationMax;
	contourLineFboOffset = elevationMax;
    contourLineFactor = contourLineFboScale/contourLineDistance;
    
    //setup the mesh
    setupMesh();
    
	// Load shaders
    bool loaded = true;
#ifdef TARGET_OPENGLES
    ofLogVerbose("GreatSand") << "setup(): Loading shadersES2";
	loaded = loaded && elevationShader.load("shadersES2/elevationShader");
	loaded = loaded && heightMapShader.load("shadersES2/heightMapShader");
#else
	if(ofIsGLProgrammableRenderer()){
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL3/elevationShader";
		loaded = loaded && elevationShader.load("shadersGL3/elevationShader");
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL3/heightMapShader";
		loaded = loaded && heightMapShader.load("shadersGL3/heightMapShader");
	}else{
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL2/elevationShader";
		loaded = loaded && elevationShader.load("shadersGL2/elevationShader");
        ofLogVerbose("GreatSand") << "setup(): Loading shadersGL2/heightMapShader";
		loaded = loaded && heightMapShader.load("shadersGL2/heightMapShader");
	}
#endif
    if (!loaded)
    {
        ofLogError("GreatSand") << "setup(): shader not loaded" ;
    }
    
    //Prepare fbo
    fboProjWindow.allocate(projResX, projResY, GL_RGBA);
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();
    
    setupGui();
    
    // Setup range, base plane and conversion matrices
    updateConversionMatrices();
    updateRangesAndBasePlane();
}

void SandSurfaceRenderer::updateConversionMatrices(){
    // Get conversion matrices
    transposedKinectProjMatrix = kinectProjector->getTransposedKinectProjMatrix();
    transposedKinectWorldMatrix = kinectProjector->getTransposedKinectWorldMatrix();
}

void SandSurfaceRenderer::updateRangesAndBasePlane(){
    basePlaneEq = kinectProjector->getBasePlaneEq();
    basePlaneNormal = kinectProjector->getBasePlaneNormal();
    basePlaneOffset = kinectProjector->getBasePlaneOffset();

    // Set the FilteredDepthImage native scale - converted to 0..1 when send to the shader
    kinectProjector->updateNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);
    
    // Calculate the  FilteredDepthImage scaling and offset coefficients
	FilteredDepthScale = elevationMin-elevationMax;
	FilteredDepthOffset = basePlaneOffset.z+elevationMax;
    
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneOffset: " << basePlaneOffset ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneNormal: " << basePlaneNormal ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneEq: " << basePlaneEq ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMin: " << elevationMin ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMax: " << elevationMax ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): heightMap.getNumEntries(): " << heightMap.getNumEntries() ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): contourLineDistance: " << contourLineDistance ;
}

void SandSurfaceRenderer::setupMesh(){
    // Initialise mesh
    //    float planeScale = 1;
    ofRectangle kinectROI = kinectProjector->getKinectROI();
    ofVec2f kinectRes = kinectProjector->getKinectRes();
    meshwidth = kinectROI.width;//kinectResX;
    meshheight = kinectROI.height;//kinectResY;
    mesh.clear();
    for(unsigned int y=0;y<meshheight;y++)
        for(unsigned int x=0;x<meshwidth;x++)
        {
            ofPoint pt = ofPoint(x*kinectRes.x/(meshwidth-1)+kinectROI.x,y*kinectRes.y/(meshheight-1)+kinectROI.y,0.0f)-ofPoint(0.5,0.5,0); // We move of a half pixel to center the color pixel (more beautiful)
            //        ofPoint pt = ofPoint(x*kinectResX*planeScale/(meshwidth-1)+kinectROI.x,y*kinectResY*planeScale/(meshheight-1)+kinectROI.y,0.0f)-kinectROI.getCenter()*planeScale+kinectROI.getCenter()-ofPoint(0.5,0.5,0); // with a planescaling
            mesh.addVertex(pt); // make a new vertex
            mesh.addTexCoord(pt);
        }
    for(unsigned int y=0;y<meshheight-1;y++)
        for(unsigned int x=0;x<meshwidth-1;x++)
        {
            mesh.addIndex(x+y*meshwidth);         // 0
            mesh.addIndex((x+1)+y*meshwidth);     // 1
            mesh.addIndex(x+(y+1)*meshwidth);     // 10
            
            mesh.addIndex((x+1)+y*meshwidth);     // 1
            mesh.addIndex((x+1)+(y+1)*meshwidth); // 11
            mesh.addIndex(x+(y+1)*meshwidth);     // 10
        }
}

void SandSurfaceRenderer::update(){
    if (drawContourLines)
        prepareContourLinesFbo();
    drawSandbox();
    colorList->update();
}

void SandSurfaceRenderer::drawMainWindow(){
    fboProjWindow.draw(0,0);
    colorList->draw();
}

void SandSurfaceRenderer::drawProjectorWindow(){
    fboProjWindow.draw(0,0);
}

void SandSurfaceRenderer::drawSandbox() {
    fboProjWindow.begin();
    ofBackground(0);
    kinectProjector->bind();
    heightMapShader.begin();
    heightMapShader.setUniformMatrix4f("kinectProjMatrix",transposedKinectProjMatrix);
    heightMapShader.setUniformMatrix4f("kinectWorldMatrix",transposedKinectWorldMatrix);
    heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
    heightMapShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    heightMapShader.setUniform4f("basePlaneEq", basePlaneEq);
    heightMapShader.setUniformTexture("heightColorMapSampler",heightMap.getTexture(), 2);
    heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 3);
    heightMapShader.setUniform1f("contourLineFactor", contourLineFactor);
    heightMapShader.setUniform1i("drawContourLines", drawContourLines);
    mesh.draw();
    heightMapShader.end();
    kinectProjector->unbind();
    fboProjWindow.end();
}

void SandSurfaceRenderer::prepareContourLinesFbo()
{
    contourLineFramebufferObject.begin();
    ofClear(255,255,255, 0);
    kinectProjector->bind();
	elevationShader.begin();
    elevationShader.setUniformMatrix4f("kinectProjMatrix",transposedKinectProjMatrix);
    elevationShader.setUniformMatrix4f("kinectWorldMatrix",transposedKinectWorldMatrix);
    elevationShader.setUniform2f("contourLineFboTransformation",ofVec2f(contourLineFboScale,contourLineFboOffset));
    elevationShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    elevationShader.setUniform4f("basePlaneEq", basePlaneEq);
    mesh.draw();
    elevationShader.end();
    kinectProjector->unbind();
    contourLineFramebufferObject.end();
}

void SandSurfaceRenderer::setupGui(){
    // instantiate and position the gui //
    gui = new ofxDatGui( ofxDatGuiAnchor::TOP_LEFT );
    
    gui->addToggle("Contour lines", drawContourLines);
    gui->addSlider("Contour lines distance", 1, 30, contourLineDistance);
    gui->addBreak();
    gui->addButton("Color #1")->setName("ColorName");
    gui->addSlider("Height", -300, 300, 0);
    gui->addColorPicker("Color", ofColor::black);
    gui->addButton("Move up");
    gui->addButton("Move down");
    gui->addButton("Insert new color after current color");
    gui->addButton("Remove color");
    gui->addButton("Reset colors");
    gui->expand();
    gui->addHeader(":: Display ::");
    gui->addFooter();
    gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    
    // once the gui has been assembled, register callbacks to listen for component specific events //
    gui->onButtonEvent(this, &SandSurfaceRenderer::onButtonEvent);
    gui->onToggleEvent(this, &SandSurfaceRenderer::onToggleEvent);
    gui->onSliderEvent(this, &SandSurfaceRenderer::onSliderEvent);
    gui->onColorPickerEvent(this, &SandSurfaceRenderer::onColorPickerEvent);
    
    // add a scroll view to list colors //
    colorList = new ofxDatGuiScrollView("Colors", 8);
    colorList->setPosition(gui->getPosition().x, gui->getPosition().y+gui->getHeight());
    colorList->onScrollViewEvent(this, &SandSurfaceRenderer::onScrollViewEvent);
    
    for (int i = 0 ; i < heightMap.size() ; i++){ // Reverse order so we have highest color on the top of the list
        int j = heightMap.size()-1-i;
        colorList->add("color");
        updateColorListColor(i, j);
        colorList->get(i)->setLabel("Height: "+ofToString(heightMap[j].height));
    }
    
    //Initiate color controls
    selectedColor = 0;
    int j = heightMap.size()-1;
    gui->getColorPicker("Color")->setColor(heightMap[j].color);
    gui->getSlider("Height")->setValue(heightMap[j].height);
    gui->getSlider("Height")->setMax(heightMap[j].height+100);
    gui->getSlider("Height")->setMin(heightMap[j-1].height);
    colorList->get(0)->setLabelAlignment(ofxDatGuiAlignment::CENTER);
}

void SandSurfaceRenderer::updateColorListColor(int i, int j){
    ofColor kc = heightMap[j].color;
    ofColor kb = kc;
    float st = (kc.getSaturation() > 10) ? kc.getSaturation()-10 : 0;
    float bt = (kc.getBrightness() < 245) ? kc.getBrightness()+10 : 255;
    kb.setSaturation(st);
    kb.setBrightness(bt);
    colorList->get(i)->setBackgroundColors(kc, kb, kb);
    colorList->get(i)->setLabelColor(kc.getInverted());
}

void SandSurfaceRenderer::onButtonEvent(ofxDatGuiButtonEvent e){
    if (e.target->is("Reset colors")) {
        if (!heightMap.loadFile("HeightColorMap.xml"))
            heightMap.createFile("HeightColorMap.xml");
        colorList->clear();
        for (int i = 0 ; i < heightMap.size() ; i++){
            int j = heightMap.size()-1-i;
            colorList->add("color");
            updateColorListColor(i, j);
            colorList->get(i)->setLabel("Height: "+ofToString(heightMap[j].height));
        }
        //Initiate color controls
        selectedColor = 0;
        int j = heightMap.size()-1;
        gui->getColorPicker("Color")->setColor(heightMap[j].color);
        gui->getSlider("Height")->setValue(heightMap[j].height);
        gui->getSlider("Height")->setMax(heightMap[j].height+100);
        gui->getSlider("Height")->setMin(heightMap[j-1].height);
        colorList->get(0)->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    } else if (e.target->is("Insert new color after current color")){
        int i = heightMap.size();
        int j = heightMap.size()-1-selectedColor;
        float newheight = (j < heightMap.size()-1) ? (heightMap[j+1].height+heightMap[j].height)/2 : heightMap[j].height+1;
        colorList->add("color");
        updateColorListColor(i, j);
        colorList->get(i)->setLabel("Height: "+ofToString(newheight));
        colorList->move(i,selectedColor+1);
        heightMap.addKey(heightMap[j].color, newheight);
    } else if (e.target->is("Remove color")){
        if (heightMap.size() > 1){
            int j = heightMap.size()-1-selectedColor;
            heightMap.removeKey(j);
            colorList->remove(selectedColor);

            int i = selectedColor;
            if (i > heightMap.size()-1){
                i -=1;
                selectedColor = i;
            }
            j = heightMap.size()-1-i;
            colorList->get(i)->setLabelAlignment(ofxDatGuiAlignment::CENTER);
            gui->getButton("ColorName")->setLabel("Color #"+ofToString(i+1));
            gui->getColorPicker("Color")->setColor(heightMap[j].color);
            ofxDatGuiSlider* hgt = gui->getSlider("Height");
            hgt->setMin(heightMap.getScalarRangeMin());
            hgt->setMax(heightMap.getScalarRangeMax());
            float nmax = (j < heightMap.size()-1) ? heightMap[j+1].height : heightMap[j].height+100;
            hgt->setMax(nmax);
            float nmin = (j > 0) ? heightMap[j-1].height : heightMap[j].height-100;
            hgt->setMin(nmin);
            hgt->setValue(heightMap[j].height);
        }
    } else if (e.target->is("Move up")){
        int i = selectedColor;
        int j = heightMap.size()-1-i;
        if (i>0){
            heightMap.swapKeys(j, j+1);
            updateColorListColor(i, j);
            updateColorListColor(i-1, j+1);
            
            colorList->get(i-1)->setLabelAlignment(ofxDatGuiAlignment::CENTER);
            colorList->get(i)->setLabelAlignment(ofxDatGuiAlignment::LEFT);
            
            gui->getButton("ColorName")->setLabel("Color #"+ofToString(i));
            
            ofxDatGuiSlider* hgt = gui->getSlider("Height");
            hgt->setMin(heightMap.getScalarRangeMin());
            hgt->setMax(heightMap.getScalarRangeMax());
            
            selectedColor -= 1;
            j += 1;
            float nmax = (j < heightMap.size()-1) ? heightMap[j+1].height : heightMap[j].height+100;
            hgt->setMax(nmax);
            float nmin = (j > 0) ? heightMap[j-1].height : heightMap[j].height-100;
            hgt->setMin(nmin);
            hgt->setValue(heightMap[j].height);
       }
    } else if (e.target->is("Move down")){
        int i = selectedColor;
        int j = heightMap.size()-1-i;
        if (j>0){
            heightMap.swapKeys(j, j-1);
            updateColorListColor(i, j);
            updateColorListColor(i+1, j-1);
            
            colorList->get(i+1)->setLabelAlignment(ofxDatGuiAlignment::CENTER);
            colorList->get(i)->setLabelAlignment(ofxDatGuiAlignment::LEFT);
            
            gui->getButton("ColorName")->setLabel("Color #"+ofToString(i+2));
            
            ofxDatGuiSlider* hgt = gui->getSlider("Height");
            hgt->setMin(heightMap.getScalarRangeMin());
            hgt->setMax(heightMap.getScalarRangeMax());
            
            selectedColor += 1;
            j -= 1;
            float nmax = (j < heightMap.size()-1) ? heightMap[j+1].height : heightMap[j].height+100;
            hgt->setMax(nmax);
            float nmin = (j > 0) ? heightMap[j-1].height : heightMap[j].height-100;
            hgt->setMin(nmin);
            hgt->setValue(heightMap[j].height);
        }
    }
}

void SandSurfaceRenderer::onToggleEvent(ofxDatGuiToggleEvent e){
    if (e.target->is("Contour lines")) {
        drawContourLines = e.checked;
    }
}

void SandSurfaceRenderer::onColorPickerEvent(ofxDatGuiColorPickerEvent e){
    if (e.target->is("Color")) {
        int i = selectedColor;
        int j = heightMap.size()-1-i;
        heightMap.setColorKey(j, e.color);
        updateColorListColor(i, j);
    }
}

void SandSurfaceRenderer::onSliderEvent(ofxDatGuiSliderEvent e){
    if (e.target->is("Contour lines distance")) {
        contourLineDistance = e.value;
        contourLineFactor = contourLineFboScale/contourLineDistance;        
    } else if (e.target->is("Height")) {
        int i = selectedColor;
        int j = heightMap.size()-1-i;
        heightMap.setHeightKey(j, e.value);
        colorList->get(i)->setLabel("Height: "+ofToString(e.value));
    }
}

void SandSurfaceRenderer::onScrollViewEvent(ofxDatGuiScrollViewEvent e){
    int i = e.index;
    if (i != selectedColor){
        int j = heightMap.size()-1-i;
        
        e.target->setLabelAlignment(ofxDatGuiAlignment::CENTER);
        colorList->get(selectedColor)->setLabelAlignment(ofxDatGuiAlignment::LEFT);
        
        gui->getButton("ColorName")->setLabel("Color #"+ofToString(i+1));
        
        gui->getColorPicker("Color")->setColor(heightMap[j].color);
        
        ofxDatGuiSlider* hgt = gui->getSlider("Height");
        hgt->setMin(heightMap.getScalarRangeMin());
        hgt->setMax(heightMap.getScalarRangeMax());
        float nmax = (j < heightMap.size()-1) ? heightMap[j+1].height : heightMap[j].height+100;
        hgt->setMax(nmax);
        float nmin = (j > 0) ? heightMap[j-1].height : heightMap[j].height-100;
        hgt->setMin(nmin);
        hgt->setValue(heightMap[j].height);
        
        selectedColor = i;
    }
}




