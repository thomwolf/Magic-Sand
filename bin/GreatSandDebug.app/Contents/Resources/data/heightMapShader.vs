#version 120

#extension GL_ARB_texture_rectangle : enable

uniform mat4 modelViewProjectionMatrix;
//in vec4 position;

void main(){
    gl_Position = modelViewProjectionMatrix * gl_Vertex;
}

//uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//uniform mat4 kinectWorldMatrix; // Transformation from kinect image space to kinect world space
//uniform mat4 kinectProjMatrix; // Transformation from kinect world space to proj image space
//uniform vec4 basePlane; // Plane equation of the base plane
//uniform vec2 heightColorMapTransformation; // Transformation from elevation to height color map texture coordinate factor and offset
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


