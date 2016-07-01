#version 120

//out vec4 outputColor;

varying float bug;
//in vec2 varyingtexcoord;

uniform sampler2DRect heightColorMapSampler;
uniform sampler2DRect pixelCornerElevationSampler; // Sampler for the half pixel texture
uniform float contourLineFactor;
uniform int drawContourLines;
//
//in vec4 heightColorMapTexCoord; // Texture coordinate for the height color map

void main()
{
    vec2 depthPos = vec2(bug, 0.5);//depthvalue*texsize, 0.5);
    vec4 color =  texture2DRect(heightColorMapSampler, depthPos);	//colormap converted depth

    if (drawContourLines == 1)
    {
        // Contour line computation
        /* Calculate the contour line interval containing each pixel corner by evaluating the half-pixel offset elevation texture: */
        float corner0=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x,gl_FragCoord.y)).r*contourLineFactor);
        float corner1=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x+1.0,gl_FragCoord.y)).r*contourLineFactor);
        float corner2=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x,gl_FragCoord.y+1.0)).r*contourLineFactor);
        float corner3=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x+1.0,gl_FragCoord.y+1.0)).r*contourLineFactor);
        
        /* Find all pixel edges that cross at least one contour line: */
        int edgeMask=0;
        int numEdges=0;
        if(corner0!=corner1)
        {
            edgeMask+=1;
            ++numEdges;
        }
        if(corner2!=corner3)
        {
            edgeMask+=2;
            ++numEdges;
        }
        if(corner0!=corner2)
        {
            edgeMask+=4;
            ++numEdges;
        }
        if(corner1!=corner3)
        {
            edgeMask+=8;
            ++numEdges;
        }
        
        /* Check for all cases in which the pixel should be colored as a topographic contour line: */
        if(numEdges>2||edgeMask==3||edgeMask==12||(numEdges==2&&mod(floor(gl_FragCoord.x)+floor(gl_FragCoord.y),2.0)==0.0))
        {
            /* Topographic contour lines are rendered in black: */
            color=vec4(0.0,0.0,0.0,1.0);
        }
    }

    gl_FragColor = color;//vec4(bug/512, 0.0, 0.0, 1.0);//color;//texture1D(heightColorMapSampler,bug);//
}

//uniform sampler2DRect pixelCornerElevationSampler;
//uniform float contourLineFactor;
//
//varying float col1; // Texture coordinate for the height color map
//varying float color2; // Texture coordinate for the height color map

//void main()
//{
//    /* Get the fragment's color from the height color map: */
//    vec4 baseColor=texture1D(heightColorMapSampler,heightColorMapTexCoord);
//
//    /* Modulate the base color by contour line color: */
//   // addContourLines(gl_FragCoord.xy,baseColor);
//
//    /* Assign the final color to the fragment: */
//    gl_FragColor = baseColor; // vec4(1.0, 0.0, 0.0, 1.0);//
//}

void addContourLines(in vec2 fragCoord,inout vec4 baseColor)
{
}

