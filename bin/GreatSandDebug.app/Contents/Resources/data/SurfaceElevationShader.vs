/***********************************************************************
SurfaceElevationShader - Shader to render the elevation of a surface
relative to a plane.
Copyright (c) 2012 Oliver Kreylos

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

#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
uniform mat4 kinectWorldMatrix; // Transformation from depth image space to camera space
uniform vec4 basePlane; // Plane equation of the base plane

varying float elevation; // Elevation relative to base plane

void main()
	{
	/* Get the vertex' depth image-space z coordinate from the texture: */
	vec4 vertexDic=gl_Vertex;
	vertexDic.z=texture2DRect(depthSampler,vertexDic.xy).r;
	
	/* Transform the vertex from depth image space to camera space: */
        vec4 vertexCc=depthProjection*vertexDic;
	
	/* Plug camera-space vertex into the base plane equation: */
	elevation=dot(basePlane,vertexCc)/vertexCc.w;
	
	/* Transform vertex to clip coordinates: */
	gl_Position=gl_ModelViewProjectionMatrix*vertexCc;
	}
