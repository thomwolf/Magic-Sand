#version 120

varying float depthfrag;

void main()
{
    gl_FragColor = vec4(depthfrag, 0.0, 0.0, 1.0); // Write the elevation directly into the frame buffer
}
