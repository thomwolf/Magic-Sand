/***********************************************************************
SurfaceRenderer - Class to render a surface defined by a regular grid in
depth image space.
Copyright (c) 2012-2015 Oliver Kreylos

This file is part of the Augmented Reality Sandbox (SARndbox).

The Augmented Reality Sandbox is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Augmented Reality Sandbox is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#pragma once

#include <IO/FileMonitor.h>
#include <Geometry/Plane.h>
#include <Geometry/ProjectiveTransformation.h>
#include "ofMain.h"

/* Forward declarations: */

class SurfaceRenderer //:public GLObject
	{
	/* Embedded classes: */
	public:
	typedef double Scalar;
	typedef Geometry::Plane<Scalar,3> Plane; // Type for planes in camera space
	typedef Geometry::ProjectiveTransformation<Scalar,3> PTransform; // Type for projective transformations
	
	private:
		ofTexture depthTexture; // ID of texture object holding surface's vertex elevations in depth image space
		unsigned int depthTextureVersion; // Version number of the depth image texture
		int contourLineFramebufferSize[2]; // Current width and height of contour line rendering frame buffer
		ofFbo contourLineFramebufferObject; // Frame buffer object used to render topographic contour lines
        ofFbo                       fbo;			//Buffer for main drawing
		unsigned int surfaceSettingsVersion; // Version number of surface settings for which the height map shader was built
	
        ofShader elevationShader, heightMapShader;            //Shaders for contourline computing and main display
        ofMesh mesh;  // Mesh
        
	/* Elements: */
//	IO::FileMonitor fileMonitor; // Monitor to watch the renderer's external shader source files
    unsigned int width, height; // Width and height of the depth image
	Plane basePlane; // Base plane to calculate surface elevation
	ofVec4f basePlaneEq; // Base plane equation in GLSL-compatible format
    PTransform depthProjection; // The transformation from depth image space to camera space
    GLfloat depthProjectionMatrix[16]; // Same, in GLSL-compatible format
	bool usePreboundDepthTexture; // Flag if the renderer should use the depth texture already bound to texture unit 0
	bool drawContourLines; // Flag if topographic contour lines are enabled
	GLfloat contourLineFactor; // Inverse elevation distance between adjacent topographic contour lines
	bool useHeightMap; // Flag whether to use a height color map for the surface
	GLfloat heightMapScale,heightMapOffset; // Scale and offset values to convert from elevation to height color map texture coordinates
	ofFloatPixels depthImage; // The most recent float-pixel depth image
	unsigned int depthImageVersion; // Version number of the depth image
	double animationTime; // Time value for water animation
    float projFactor; // To convert kinect camera coord to world coord

	/* Constructors and destructors: */
	public:
	SurfaceRenderer(const unsigned int swidth,const unsigned int sheight,const PTransform& sDepthProjection,const Plane& sBasePlane); // Creates a renderer for the given image size, depth projection, and base plane
	
	/* Methods from GLObject: */
	virtual void initContext();
	
	/* New methods: */
	void setUsePreboundDepthTexture(bool newUsePreboundDepthTexture); // Enables or disables using a pre-bound depth texture
	void setDrawContourLines(bool newDrawContourLines); // Enables or disables topographic contour lines
	void setContourLineDistance(GLfloat newContourLineDistance); // Sets the elevation distance between adjacent topographic contour lines
	void setUseHeightMap(bool newUseHeightMap); // Enable or disable height-based surface coloring
	void setHeightMapRange(GLsizei newHeightMapSize,GLfloat newMinElevation,GLfloat newMaxElevation); // Sets the elevation range for height color mapping
	void setDepthImage(ofFloatPixels newDepthImage); // Sets a new depth image for subsequent surface rendering
	void setAnimationTime(double newAnimationTime); // Sets the time for water animation in seconds
	void glPrepareContourLines(); // Prepares to render topographic contour lines by rendering the base plane-relative elevations of pixel corners into a frame buffer
	void glRenderSinglePass(ofTexture heightColorMapTexture); // Renders the surface in a single pass using the current surface settings
	};
