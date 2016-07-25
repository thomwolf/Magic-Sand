#version 150

out vec4 outputColor;

in float depthfrag;

void main()
{
    outputColor = vec4(depthfrag, 0.0, 0.0, 1.0); // Write the elevation directly into the frame buffer
}

