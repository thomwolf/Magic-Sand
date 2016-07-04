//
//  SandSurfaceRenderer.cpp
//  GreatSand
//
//  Created by Thomas Wolf on 02/07/16.
//
//

#include "SandSurfaceRenderer.h"

using namespace ofxCSG;

void SandSurfaceRenderer::setup(int sprojResX, int sprojResY){
    // Sandbox contourlines
    drawContourLines = true; // Flag if topographic contour lines are enabled
	contourLineDistance = 10.0; // Elevation distance between adjacent topographic contour lines in millimiters
    
    // Initialize the fbos and images
    projResX = sprojResX;
    projResY = sprojResY;
    contourLineFramebufferObject.allocate(projResX+1, projResY+1, GL_RGBA);
    contourLineFramebufferObject.begin();
    ofClear(0,0,0,255);
    contourLineFramebufferObject.end();

    // Load colormap and set heightmap
    heightMap.load("HeightColorMap.yml");
    
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
    
    fboProjWindow.allocate(projResX, projResY, GL_RGBA);
    fboProjWindow.begin();
    ofClear(0,0,0,255);
    fboProjWindow.end();

    // Setup elevation ranges and base plane equation
    setRangesAndBasePlaneEquation();
}

//--------------------------------------------------------------
void SandSurfaceRenderer::setRangesAndBasePlaneEquation(){
    basePlaneEq = kinectProjector->getBasePlaneEq();
    basePlaneNormal = kinectProjector->getBasePlaneNormal();
    basePlaneOffset = kinectProjector->getBasePlaneOffset();

    //Set elevation Min and Max
    elevationMin = -heightMap.getScalarRangeMin();
    elevationMax = -heightMap.getScalarRangeMax();
    
    // Set the FilteredDepthImage native scale - converted to 0..1 when send to the shader
    kinectProjector->updateNativeScale(basePlaneOffset.z+elevationMax, basePlaneOffset.z+elevationMin);
    
    // Calculate the  height map elevation scaling and offset coefficients
	heightMapScale = (heightMap.getNumEntries()-1)/((elevationMax-elevationMin));
	heightMapOffset = 0.5/heightMap.getNumEntries()-heightMapScale*elevationMin;
    
    // Calculate the  FilteredDepthImage scaling and offset coefficients
	FilteredDepthScale = elevationMin-elevationMax;
	FilteredDepthOffset = basePlaneOffset.z+elevationMax;
    
    // Calculate the contourline fbo scaling and offset coefficients
	contourLineFboScale = elevationMin-elevationMax;
	contourLineFboOffset = elevationMax;
    contourLineFactor = contourLineFboScale/(contourLineDistance);
    
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneOffset: " << basePlaneOffset ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneNormal: " << basePlaneNormal ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): basePlaneEq: " << basePlaneEq ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMin: " << elevationMin ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): elevationMax: " << elevationMax ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): heightMap.getNumEntries(): " << heightMap.getNumEntries() ;
    ofLogVerbose("GreatSand") << "setRangesAndBasePlaneEquation(): contourLineDistance: " << contourLineDistance ;
}

//--------------------------------------------------------------
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
}

void SandSurfaceRenderer::draw(){
    fboProjWindow.draw(0,0);
}

void SandSurfaceRenderer::drawSandbox() {
    fboProjWindow.begin();
    ofBackground(0);
    kinectProjector->getTexture().bind();
    heightMapShader.begin();
    heightMapShader.setUniformMatrix4f("kinectProjMatrix",kinectProjector->getTransposedKinectProjMatrix());
    heightMapShader.setUniformMatrix4f("kinectWorldMatrix",kinectProjector->getTransposedKinectWorldMatrix());
    heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
    heightMapShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    heightMapShader.setUniform4f("basePlaneEq", basePlaneEq);
    heightMapShader.setUniformTexture("heightColorMapSampler",heightMap.getTexture(), 2);
    heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 3);
    heightMapShader.setUniform1f("contourLineFactor", contourLineFactor);
    heightMapShader.setUniform1i("drawContourLines", drawContourLines);
    mesh.draw();
    heightMapShader.end();
    kinectProjector->getTexture().unbind();
    fboProjWindow.end();
}

void SandSurfaceRenderer::prepareContourLinesFbo()
{
    contourLineFramebufferObject.begin();
    ofClear(255,255,255, 0);
    kinectProjector->getTexture().bind();
	elevationShader.begin();
    elevationShader.setUniformMatrix4f("kinectProjMatrix",kinectProjector->getTransposedKinectProjMatrix());
    elevationShader.setUniformMatrix4f("kinectWorldMatrix",kinectProjector->getTransposedKinectWorldMatrix());
    elevationShader.setUniform2f("contourLineFboTransformation",ofVec2f(contourLineFboScale,contourLineFboOffset));
    elevationShader.setUniform2f("depthTransformation",ofVec2f(FilteredDepthScale,FilteredDepthOffset));
    elevationShader.setUniform4f("basePlaneEq", basePlaneEq);
    mesh.draw();
    elevationShader.end();
    kinectProjector->getTexture().unbind();
    contourLineFramebufferObject.end();
}


