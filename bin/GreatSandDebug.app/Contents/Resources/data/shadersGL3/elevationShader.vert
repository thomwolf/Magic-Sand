#version 150

// these are for the programmable pipeline system
uniform mat4 modelViewProjectionMatrix;
in vec4 position;

//uniform float mouseRange;
//uniform vec2 mousePos;
//uniform vec4 mouseColor;
//
void main()
{
    // copy position so we can work with it.
    vec4 pos = position;
    
//    // direction vector from mouse position to vertex position.
//	vec2 dir = pos.xy - mousePos;
//    
//    // distance between the mouse position and vertex position.
//	float dist =  sqrt(dir.x * dir.x + dir.y * dir.y);
//    
//    // check vertex is within mouse range.
//	if(dist > 0.0 && dist < mouseRange) {
//		
//		// normalise distance between 0 and 1.
//		float distNorm = dist / mouseRange;
//        
//		// flip it so the closer we are the greater the repulsion.
//		distNorm = 1.0 - distNorm;
//		
//        // make the direction vector magnitude fade out the further it gets from mouse position.
//        dir *= distNorm;
//        
//		// add the direction vector to the vertex position.
//		pos.x += dir.x;
//		pos.y += dir.y;
//	}
    
	// finally set the pos to be that actual position rendered
	gl_Position = modelViewProjectionMatrix * pos;
}

//uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//uniform mat4 kinectWorldMatrix; // Transformation from kinect image space to kinect world space
//uniform mat4 kinectProjMatrix; // Transformation from kinect world space to proj image space
//uniform vec4 basePlane; // Plane equation of the base plane
//
//varying float elevation; // Elevation relative to base plane
//
//void main()
//{
//	/* Get the vertex' depth image-space z coordinate from the texture: */
//	vec4 vertexDic=gl_Vertex;
//	vertexDic.z=texture2DRect(depthSampler,vertexDic.xy).r;
//	
//	/* Transform the vertex from depth image space to camera space: */
//    vec4 vertexCc=kinectWorldMatrix*vertexDic*vertexDic.z;
//    vertexCc.w = 1.0;
//	
//	/* Plug camera-space vertex into the base plane equation: */
//	elevation=dot(basePlane,vertexCc);
//	
//	/* Transform vertex to clip coordinates: */
//    vec4 screenVertex=kinectProjMatrix*vertexCc;
//    gl_Position=screenVertex/screenVertex.z;
//
//}
