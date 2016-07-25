#version 120

varying float depthfrag;

uniform sampler2DRect tex0; // Sampler for the depth image-space elevation texture
uniform vec2 depthTransformation; // Normalisation factor and offset applied by openframeworks
uniform vec2 contourLineFboTransformation; // Transformation from elevation to normalized contourline fbo unit factor and offset

uniform mat4 kinectWorldMatrix; // Transformation from kinect image space to kinect world space
uniform mat4 kinectProjMatrix; // Transformation from kinect world space to proj image space
uniform vec4 basePlaneEq; // Base plane equation

void main()
{
    vec4 position = gl_Vertex;
    vec2 texcoord = gl_MultiTexCoord0.xy;
    // copy position so we can work with it.
    vec4 pos = position;
    vec2 varyingtexcoord = pos.xy;//texcoord;
    
    /* Set the vertex' depth image-space z coordinate from the texture: */
    vec4 texel0 = texture2DRect(tex0, varyingtexcoord);
    float depth1 = texel0.r;
    float depth = depth1 * depthTransformation.x + depthTransformation.y;
    
    pos.z = depth;
    pos.w = 1;
    
    /* Transform the vertex from depth image space to world space: */
    vec4 vertexCc = kinectWorldMatrix * pos;  // Transposed multiplication (Row-major order VS col major order
    vec4 vertexCcx = vertexCc * depth;
    vertexCcx.w = 1;
    
    /* Take into account baseplane orientation and location: */
    float elevation = dot(basePlaneEq,vertexCcx);///vertexCc.w;
    depthfrag = (elevation-contourLineFboTransformation.y)/contourLineFboTransformation.x;
    
    /* Transform vertex to proj coordinates: */
    vec4 screenPos = kinectProjMatrix * vertexCcx;
    vec4 projectedPoint = screenPos / screenPos.z;
    
    projectedPoint.z = 0;
    projectedPoint.w = 1;
    
	gl_Position = gl_ModelViewProjectionMatrix * projectedPoint;
}
