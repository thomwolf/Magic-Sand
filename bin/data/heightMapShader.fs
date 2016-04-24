#extension GL_ARB_texture_rectangle : enable

//uniform sampler2DRect pixelCornerElevationSampler;
uniform float contourLineFactor;
uniform sampler1D heightColorMapSampler;

varying float heightColorMapTexCoord; // Texture coordinate for the height color map
//varying float col1; // Texture coordinate for the height color map
//varying float color2; // Texture coordinate for the height color map

//void addContourLines(in vec2 fragCoord,inout vec4 baseColor)
//{
//	/* Calculate the contour line interval containing each pixel corner by evaluating the half-pixel offset elevation texture: */
//	float corner0=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y)).r*contourLineFactor);
//	float corner1=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y)).r*contourLineFactor);
//	float corner2=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x,fragCoord.y+1.0)).r*contourLineFactor);
//	float corner3=floor(texture2DRect(pixelCornerElevationSampler,vec2(fragCoord.x+1.0,fragCoord.y+1.0)).r*contourLineFactor);
//	
//	/* Find all pixel edges that cross at least one contour line: */
//	int edgeMask=0;
//	int numEdges=0;
//	if(corner0!=corner1)
//    {
//		edgeMask+=1;
//		++numEdges;
//    }
//	if(corner2!=corner3)
//    {
//		edgeMask+=2;
//		++numEdges;
//    }
//	if(corner0!=corner2)
//    {
//		edgeMask+=4;
//		++numEdges;
//    }
//	if(corner1!=corner3)
//    {
//		edgeMask+=8;
//		++numEdges;
//    }
//	
//	/* Check for all cases in which the pixel should be colored as a topographic contour line: */
//	if(numEdges>2||edgeMask==3||edgeMask==12||(numEdges==2&&mod(floor(fragCoord.x)+floor(fragCoord.y),2.0)==0.0))
//    {
//		/* Topographic contour lines are rendered in black: */
//		baseColor=vec4(0.0,0.0,0.0,1.0);
//    }
//}

void main()
{
    /* Get the fragment's color from the height color map: */
    vec4 baseColor=texture1D(heightColorMapSampler,vec4(color1, 0.0, 0.0, 1.0));//heightColorMapTexCoord);
    
    /* Modulate the base color by contour line color: */
   // addContourLines(gl_FragCoord.xy,baseColor);
    
    /* Assign the final color to the fragment: */
    gl_FragColor=vec4(1.0, 0.0, 0.0, 1.0);//baseColor;
}
