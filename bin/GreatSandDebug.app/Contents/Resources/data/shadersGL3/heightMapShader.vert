#version 150

// these are for the programmable pipeline system and are passed in
// by default from OpenFrameworks
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 textureMatrix;
uniform mat4 modelViewProjectionMatrix;

in vec4 position;
in vec4 color;
in vec4 normal;
in vec2 texcoord;
// this is the end of the default functionality

// this is something we're creating for this shader
out vec2 varyingtexcoord;

//uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//uniform vec2 heightColorMapTransformation; // Transformation from elevation to height color map texture coordinate factor and offset
//out vec4 heightColorMapTexCoord; // Texture coordinate for the height color map

void main()
{
    // copy position so we can work with it.
    vec4 pos = position;
    varyingtexcoord = pos.xy;//texcoord;
    
//    vec4 elevationcolor=texture(depthSampler,texCoordVarying);
//    
//    heightColorMapTexCoord=elevationcolor;//*heightColorMapTransformation.x+heightColorMapTransformation.y;
//
//	// finally set the pos to be that actual position rendered
	gl_Position = modelViewProjectionMatrix * pos;
}

//uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//uniform mat4 kinectWorldMatrix; // Transformation from kinect image space to kinect world space
//uniform mat4 kinectProjMatrix; // Transformation from kinect world space to proj image space
//uniform vec4 basePlane; // Plane equation of the base plane
//
//varying float heightColorMapTexCoord; // Texture coordinate for the height color map
////varying float col1; // Texture coordinate for the height color map
////varying float color2; // Texture coordinate for the height color map
//
//void main()
//{
//    /* Get the vertex' depth image-space z coordinate from the texture: */
//    vec4 vertexDic=gl_Vertex;
//    vertexDic.z=texture2DRect(depthSampler,vertexDic.xy).r;
//    
//    /* Transform the vertex from depth image space to world space: */
//    vec4 vertexCc=kinectWorldMatrix*vertexDic*vertexDic.z;
//    vertexCc.w = 1.0;
//    
//    /* Plug camera-space vertex into the base plane equation: */
//    float elevation=dot(basePlane,vertexCc);
//    
//    /* Transform elevation to height color map texture coordinate: */
//    heightColorMapTexCoord=elevation*heightColorMapTransformation.x+heightColorMapTransformation.y;
//    
//    /* Transform vertex to clip coordinates: */
//    vec4 screenVertex=kinectProjMatrix*vertexCc;
//    gl_Position=screenVertex/screenVertex.z;
//}

//#version 150
//
//// this is how we receive the texture
//uniform sampler2DRect tex0;
//
//in vec2 texCoordVarying;
//
//out vec4 outputColor;
//
//void main()
//{
//    outputColor = texture(tex0, texCoordVarying);
//}
