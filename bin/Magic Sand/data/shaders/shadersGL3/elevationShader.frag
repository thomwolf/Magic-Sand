#version 150

out vec4 outputColor;

in float bug;

void main()
{
    outputColor = vec4(bug, 0.0, 0.0, 1.0); // Write the elevation directly into the frame buffer
}

//varying float elevation; // Elevation relative to base plane
//
//void main()
//	{
//	/* Write the elevation directly into the frame buffer: */
//	gl_FragColor=vec4(elevation,0.0,0.0,1.0);
//	}
