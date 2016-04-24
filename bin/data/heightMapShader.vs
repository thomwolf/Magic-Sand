//#version 120
//#extension GL_ARB_texture_rectangle : enable
//#extension GL_EXT_gpu_shader4 : enable
//
//void main()
//{
//        gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
//        gl_TexCoord[0] = gl_MultiTexCoord0;
//        gl_FrontColor =  gl_Color;
//}
//
#version 120
#extension GL_ARB_texture_rectangle : enable


uniform sampler2DRect depthSampler; // Sampler for the depth image-space elevation texture
//uniform mat4 depthProjection; // Transformation from depth image space to camera space
uniform vec4 basePlane; // Plane equation of the base plane
uniform vec2 heightColorMapTransformation; // Transformation from elevation to height color map texture coordinate

varying float heightColorMapTexCoord; // Texture coordinate for the height color map
//varying float col1; // Texture coordinate for the height color map
//varying float color2; // Texture coordinate for the height color map

void main()
{
    /* Get the vertex' depth image-space z coordinate from the texture: */
    vec4 vertexDic=gl_Vertex;
    vertexDic.z=texture2DRect(depthSampler,vertexDic.xy).r;
    
    /* Transform the vertex from depth image space to camera space: */
    vec4 vertexCc=/*depthProjection**/vertexDic;
    
    /* Plug camera-space vertex into the base plane equation: */
    float elevation=dot(basePlane,vertexCc)*vertexCc.z;///vertexCc.w;
    
    /* Transform elevation to height color map texture coordinate: */
//    col1 = 0.0;
//    if (vertexDic.x > 640)
//        col1 = 1.0;
    
//    color2 = 0.0;
//    if (vertexDic.y > 480)
//        color2 = 1.0;
//    
    heightColorMapTexCoord=elevation*heightColorMapTransformation.x+heightColorMapTransformation.y;
    
    /* Transform vertex to clip coordinates: */
    gl_Position=gl_ModelViewProjectionMatrix*vertexCc;
}


