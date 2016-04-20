/***********************************************************************
 SurfaceRenderer - Class to render a surface defined by a regular grid in
 depth image space.
 Adapted from the Augmented Reality Sandbox (SARndbox) by Oliver Kreylos
 ***********************************************************************/

#include "SurfaceRenderer.h"

#include <string>
#include <vector>
#include <iostream>

#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480

SurfaceRenderer::SurfaceRenderer(const unsigned int swidth,const unsigned int sheight, const PTransform& sDepthProjection, const Plane& sBasePlane)
:basePlane(sBasePlane),
usePreboundDepthTexture(false),
drawContourLines(true),contourLineFactor(1.0f),
useHeightMap(true),heightMapScale(1.0f),heightMapOffset(0.0f),
surfaceSettingsVersion(1),
depthImageVersion(1),
animationTime(0.0)
{
	
	/* Copy the depth image size: */
    width = swidth;
    height = sheight;
    
    
    /* Check if the depth projection matrix retains right-handedness: */
    PTransform::Point p1=depthProjection.transform(PTransform::Point(0,0,0));
    PTransform::Point p2=depthProjection.transform(PTransform::Point(1,0,0));
    PTransform::Point p3=depthProjection.transform(PTransform::Point(0,1,0));
    PTransform::Point p4=depthProjection.transform(PTransform::Point(0,0,1));
    bool depthProjectionInverts=Geometry::cross(p2-p1,p3-p1)*(p4-p1)<Scalar(0);
    
    /* Convert the depth projection matrix to column-major OpenGL format: */
    GLfloat* dpmPtr=depthProjectionMatrix;
    for(int j=0;j<4;++j)
        for(int i=0;i<4;++i,++dpmPtr)
            *dpmPtr=depthProjection.getMatrix()(i,j);
    
    
	/* Convert the base plane to a homogeneous plane equation: */
	for(int i=0;i<3;++i)
		basePlaneEq[i]=GLfloat(basePlane.getNormal()[i]);
	basePlaneEq[3]=GLfloat(-basePlane.getOffset());
	
	/* Initialize the depth image: */
    depthImage.allocate(width, height, 1);
    depthImage.set(0);
    initContext();
}

void SurfaceRenderer::initContext()
{
	
    // Initialise mesh
    mesh.clear();
 	for(unsigned int y=0;y<height;++y)
		for(unsigned int x=0;x<width;++x)
        {
            mesh.addVertex(ofPoint(float(x)+0.5f,float(y)+0.5f,0.0f)); // make a new vertex
        }
    for(unsigned int y=1;y<height;++y)
		for(unsigned int x=0;x<width;++x)
        {
            mesh.addIndex(y*width+x);
            mesh.addIndex((y-1)*width+x);
        }
	
	// Initialize the depth image texture:
    depthTexture.allocate(depthImage);
	
	// Load shaders
    elevationShader.load("SurfaceElevationShader.vs", "SurfaceElevationShader.fs" );
    heightMapShader.load("heightMapShader.vs", "heightMapShader.fs");
    
	// Initialize the fbos
    fbo.allocate(800, 600, GL_RGBA); // with alpha, 8 bits red, 8 bits green, 8 bits blue, 8 bits alpha, from 0 to 255 in 256 steps
    contourLineFramebufferObject.allocate(801, 601, GL_RGBA);
}

void SurfaceRenderer::setUsePreboundDepthTexture(bool newUsePreboundDepthTexture)
{
	usePreboundDepthTexture=newUsePreboundDepthTexture;
}

void SurfaceRenderer::setDrawContourLines(bool newDrawContourLines)
{
	drawContourLines=newDrawContourLines;
	++surfaceSettingsVersion;
}

void SurfaceRenderer::setContourLineDistance(GLfloat newContourLineDistance)
{
	/* Set the new contour line factor: */
	contourLineFactor=1.0f/newContourLineDistance;
}

void SurfaceRenderer::setUseHeightMap(bool newUseHeightMap)
{
	useHeightMap=newUseHeightMap;
	++surfaceSettingsVersion;
}

void SurfaceRenderer::setHeightMapRange(GLsizei newHeightMapSize,GLfloat newMinElevation,GLfloat newMaxElevation)
{
	/* Calculate the new height map elevation scaling and offset coefficients: */
	GLdouble hms=GLdouble(newHeightMapSize-1)/((newMaxElevation-newMinElevation)*GLdouble(newHeightMapSize));
	GLdouble hmo=0.5/GLdouble(newHeightMapSize)-hms*newMinElevation;
	
	heightMapScale=GLfloat(hms);
	heightMapOffset=GLfloat(hmo);
}

void SurfaceRenderer::setDepthImage(const ofFloatPixels newDepthImage)
{
	/* Update the depth image: */
	depthImage=newDepthImage;
	++depthImageVersion;
}

void SurfaceRenderer::setAnimationTime(double newAnimationTime)
{
	/* Set the new animation time: */
	animationTime=newAnimationTime;
	
}

void SurfaceRenderer::glPrepareContourLines()
{
	/*********************************************************************
     Prepare the half-pixel-offset frame buffer for subsequent per-fragment
     Marching Squares contour line extraction.
     *********************************************************************/
	
//	/* Query the current viewport: */
//	GLint viewport[4];
//	glGetIntegerv(GL_VIEWPORT,viewport);
//	
//	/* Save the currently-bound frame buffer and clear color: */
//	GLint currentFrameBuffer;
//	glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT,&currentFrameBuffer);
//	GLfloat currentClearColor[4];
//	glGetFloatv(GL_COLOR_CLEAR_VALUE,currentClearColor);
//	
//	/* Check if the contour line frame buffer needs to be created or resized */
//    if (!contourLineFramebufferObject.isAllocated()||contourLineFramebufferSize[0]!=viewport[2]+1||contourLineFramebufferSize[1]!=viewport[3]+1) {
//        for(int i=0;i<2;++i)
//            contourLineFramebufferSize[i]=viewport[2+i]+1;
//        contourLineFramebufferObject.allocate(contourLineFramebufferSize[0], contourLineFramebufferSize[1]);
//    }
//	
//	/* Extend the viewport to render the corners of the final pixels: */
//	glViewport(0,0,viewport[2]+1,viewport[3]+1);
//	glClearColor(0.0f,0.0f,0.0f,1.0f);
//	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	
	/* Adjust the projection matrix to render the corners of the final pixels: */
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX,proj);
	GLdouble xs=GLdouble(800)/GLdouble(801);
	GLdouble ys=GLdouble(600)/GLdouble(601);
	for(int j=0;j<4;++j)
    {
		proj[j*4+0]*=xs;
		proj[j*4+1]*=ys;
    }
	glLoadIdentity();
	glMultMatrixd(proj);
	
	/*********************************************************************
     Render the surface's elevation into the half-pixel offset frame
     buffer.
     *********************************************************************/
	
	/* start the elevation shader and contourLineFramebufferObject: */
    contourLineFramebufferObject.begin();
    ofClear(255,255,255, 0);
	elevationShader.begin();
	
	/* Set up the depth image texture: */
	if(!usePreboundDepthTexture)
    {
		
		/* Check if the texture is outdated: */
		if(depthTextureVersion!=depthImageVersion)
        {
			/* Upload the new depth texture: */
            depthTexture.loadData(depthImage);
			//glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,0,0,0,width,height,GL_LUMINANCE,GL_FLOAT,depthImage.getBuffer());
			
			/* Mark the depth texture as current: */
			depthTextureVersion=depthImageVersion;
        }
    }
	
    elevationShader.setUniformTexture( "depthSampler", depthTexture, 1 ); //"1" means that it is texture 1
    elevationShader.setUniformMatrix4f("depthProjection",depthProjectionMatrix);
    elevationShader.setUniform4f("basePlane",basePlaneEq);
	
	/* Draw the surface: */
    mesh.draw();
	
    elevationShader.end();
    contourLineFramebufferObject.end();
	
	/*********************************************************************
     Restore previous OpenGL state.
     *********************************************************************/
	
	/* Restore the original viewport and projection matrix: */
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
//	glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
//	
//	/* Restore the original clear color and frame buffer binding: */
//	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,currentFrameBuffer);
//	glClearColor(currentClearColor[0],currentClearColor[1],currentClearColor[2],currentClearColor[3]);
}

void SurfaceRenderer::glRenderSinglePass(ofTexture heightColorMapTexture)
{
    
	/* Check if contour line rendering is enabled: */
	if(drawContourLines)
    {
		/* Run the first rendering pass to create a half-pixel offset texture of surface elevations: */
//		glPrepareContourLines();
//        contourLineFramebufferObject.allocate(800, 600);
    }
    
	/* Bind the single-pass surface shader: */
    fbo.begin();
    ofClear(255,255,255, 0);
    heightMapShader.begin();
	
	/* Set up the depth image texture: */
	if(!usePreboundDepthTexture)
    {
		
		/* Check if the texture is outdated: */
		if(depthTextureVersion!=depthImageVersion)
        {
			/* Upload the new depth texture: */
            depthTexture.loadData(depthImage);
			
			/* Mark the depth texture as current: */
			depthTextureVersion=depthImageVersion;
        }
    }
//    uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//    uniform mat4 depthProjection; // Transformation from depth image space to camera space
//    uniform vec4 basePlane; // Plane equation of the base plane
//    uniform vec2 heightColorMapTransformation; // Transformation from elevation to height color map texture coordinate
//    uniform sampler2DRect pixelCornerElevationSampler;
//    uniform float contourLineFactor;
//    uniform sampler1D heightColorMapSampler;
//    
//    varying float heightColorMapTexCoord; // Texture coordinate for the height color map

    heightMapShader.setUniformTexture( "depthSampler", depthTexture, 1 ); //"1" means that it is texture 1
    heightMapShader.setUniformMatrix4f("depthProjection",depthProjectionMatrix);
    heightMapShader.setUniform4f("basePlane",basePlaneEq);
    heightMapShader.setUniform2f("heightColorMapTransformation",ofVec2f(heightMapScale,heightMapOffset));
//    heightMapShader.setUniformTexture("pixelCornerElevationSampler", contourLineFramebufferObject.getTexture(), 2);
    heightMapShader.setUniform1f("contourLineFactor",contourLineFactor);
    heightMapShader.setUniformTexture("heightColorMapSampler",heightColorMapTexture, 3);
    
	/* Draw the surface: */
    mesh.draw();
    heightMapShader.end();
    fbo.end();
    fbo.draw(0,0);
}

