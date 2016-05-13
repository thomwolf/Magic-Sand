#version 150

out vec4 outputColor;

void main()
{
    // gl_FragCoord contains the window relative coordinate for the fragment.
    // we use gl_FragCoord.x position to control the red color value.
    // we use gl_FragCoord.y position to control the green color value.
    // please note that all r, g, b, a values are between 0 and 1.
    
    float windowWidth = 400;
    float windowHeight = 300;
    
	float r = gl_FragCoord.x / windowWidth;
	float g = gl_FragCoord.y / windowHeight;
	float b = 1.0;
	float a = 1.0;
	outputColor = vec4(r, g, b, a);
}

//varying float elevation; // Elevation relative to base plane
//
//void main()
//	{
//	/* Write the elevation directly into the frame buffer: */
//	gl_FragColor=vec4(elevation,0.0,0.0,1.0);
//	}
