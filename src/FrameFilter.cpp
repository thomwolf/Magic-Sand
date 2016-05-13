/***********************************************************************
 FrameFilter - Class to filter streams of depth frames arriving from a
 depth camera, with code to detect unstable values in each pixel, and
 fill holes resulting from invalid samples.
 Forked from Oliver Kreylos's Augmented Reality Sandbox (SARndbox).
 ***********************************************************************/

#include "FrameFilter.h"
#include "ofConstants.h"

/****************************
 Methods of class FrameFilter:
 ****************************/

using namespace ofxCSG;

FrameFilter::FrameFilter(): newFrame(true), bufferInitiated(false)
{
}

bool FrameFilter::setup(const unsigned int swidth,const unsigned int sheight,float sdepthNorm, int sgradFieldresolution, float snearclip, float sfarclip, const ofVec3f sbasePlaneNormal, double newMinElevation,double newMaxElevation)
{
	/* Settings variables : */
	width = swidth;
    height = sheight;
    gradFieldresolution = sgradFieldresolution;
	
	/* Initialize the input frame slot: */
	inputFrameVersion=0;
	
	/* Initialize the valid depth range: */
    //	setValidDepthInterval(0U,2046U);
    
    //Save the depth normalization coef
    depthNorm = sdepthNorm;
    
	/* Initialize the stability criterion and averaging buffer */
    numAveragingSlots = 30;
    minNumSamples=(numAveragingSlots+1)/2;
    depthNorm = 2000;
    maxVariance = 4 / depthNorm/depthNorm;
    hysteresis = 0.1f / depthNorm;
	retainValids=true;
	instableValue=0.0;
    maxgradfield = 1000;
    unvalidValue = 10; // We never reach 10 since we divide the kinect raw values (0-4000) by depthNorm
    
    nearclip = snearclip;
    farclip = sfarclip;
    depthrange = sfarclip-snearclip;
    
    //    minNumSamples=(numAveragingSlots+1)/2;
    //    maxVariance=newMaxVariance;
    //    hysteresis=newHysteresis;
	
	/* Enable spatial filtering: */
    //	spatialFilter=true;
    spatialFilter = false;
    
	/* Convert the base plane equation from camera space to depth-image space: */
    //	PTransform::HVector basePlaneCc(basePlane.getNormal());
    //	basePlaneCc[3]=-basePlane.getOffset();
    //	PTransform::HVector basePlaneDic(depthProjection.getMatrix().transposeMultiply(basePlaneCc));
    //	basePlaneDic/=Geometry::mag(basePlaneDic.toVector());
    basePlaneNormal =sbasePlaneNormal;
    minPlane = newMinElevation;
    maxPlane = newMaxElevation;
	
	/* Initialize the gradient field vector*/
    gradFieldresolution = sgradFieldresolution;
    std::cout<< "Gradient Field resolution" << gradFieldresolution <<std::endl;
    gradFieldcols = width / sgradFieldresolution;
    std::cout<< "Width: " << width << " Cols: " << gradFieldcols <<std::endl;
    gradFieldrows = height / sgradFieldresolution;
    std::cout<< "Height: " << height << " Rows: " << gradFieldrows <<std::endl;
    
    //setting buffers
	initiateBuffers();
    
	return true;
}

FrameFilter::~FrameFilter(){
	// when the class is destroyed
	// close both channels and wait for
	// the thread to finish
    //	toAnalyze.close();
    //	analyzed.close();
    delete[] averagingBuffer;
    delete[] statBuffer;
    delete[] validBuffer;
    delete[] wrldcoordbuffer;
    delete[] gradField;
    //	waitForThread(true);
}

void FrameFilter::resetBuffers(void){
	/* Release all allocated buffers if needed */
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] wrldcoordbuffer;
        delete[] gradField;
    }
    initiateBuffers();
}


void FrameFilter::initiateBuffers(void){
    //    /* Initialize the input frame slot: */
    //    inputFrameVersion=0;
    
    averagingBuffer=new float[numAveragingSlots*height*width];
    float* abPtr=averagingBuffer;
    for(int i=0;i<numAveragingSlots;++i)
        for(unsigned int y=0;y<height;++y)
            for(unsigned int x=0;x<width;++x,++abPtr)
                *abPtr=unvalidValue; // Mark sample as invalid
    
    averagingSlotIndex=0;
    
    /* Initialize the statistics buffer: */
    statBuffer=new float[height*width*3];
    float* sbPtr=statBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x)
            for(int i=0;i<3;++i,++sbPtr)
                *sbPtr=0.0;
    
    /* Initialize the valid buffer: */
    validBuffer=new float[height*width];
    float* vbPtr=validBuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x,++vbPtr)
            *vbPtr=0.0;
    
    /* Initialize the gradient field buffer: */
    gradField = new ofVec2f[gradFieldcols*gradFieldrows];
    ofVec2f* gfPtr=gradField;
    for(unsigned int y=0;y<gradFieldrows;++y)
        for(unsigned int x=0;x<gradFieldcols;++x,++gfPtr)
            *gfPtr=ofVec2f(0);
    
    /* Initialize the gradient field buffer: */
    wrldcoordbuffer = new Point3f[height*width];
    Point3f* wcPtr=wrldcoordbuffer;
    for(unsigned int y=0;y<height;++y)
        for(unsigned int x=0;x<width;++x,++wcPtr)
            *wcPtr=Point3f(0, 0, 0);
    
    bufferInitiated = true;
    firstImageReady = false;
}
void FrameFilter::setDepthRange(float snearclip, float sfarclip){
    // send the frame to the thread for analyzing
    // this makes a copy but we can't avoid it anyway if
    // we want to update the grabber while analyzing
    // previous frames
    //    ++inputFrameVersion;
    //    toAnalyze.send(inputframe);
    nearclip = snearclip;
    farclip = sfarclip;
    depthrange = sfarclip-snearclip;
}

bool FrameFilter::isFrameNew(){
    return newFrame;
}

ofVec2f FrameFilter::getGradFieldXY(int x, int y){
    int xbin = (int)floorf(((float)x)/(float)gradFieldresolution);
    int ybin = (int)floorf(((float)y)/(float)gradFieldresolution);
    return gradField[ybin*gradFieldcols+xbin];
}

ofVec2f* FrameFilter::getGradField(){
    return gradField;
}

Point3f* FrameFilter::getWrldcoordbuffer(){
    return wrldcoordbuffer;
}

void FrameFilter::setROI(ofRectangle ROI){
    minX = (int) ROI.getMinX();
    maxX = (int) ROI.getMaxX();
    minY = (int) ROI.getMinY();
    maxY = (int) ROI.getMaxY();
    ROIwidth = maxX-minX;
    ROIheight = maxY-minY;
}

bool FrameFilter::isInsideROI(int x, int y){
    bool result = true;
    if (x<minX||x>maxX||y<minY||y>maxY)
        result = false;
    return result;
}

void FrameFilter::displayFlowField()
{
    
    /*
     Uncomment to draw an x at the centre of the screen
     ofPoint screenCenter = ofPoint(ofGetWidth() / 2,ofGetHeight() / 2);
     ofSetColor(255,0,0,255);
     ofLine(screenCenter.x - 50,screenCenter.y-50,screenCenter.x+50,screenCenter.y+50);
     ofLine(screenCenter.x + 50,screenCenter.y-50,screenCenter.x-50,screenCenter.y+50);
     */
    
    for(int rowPos=0; rowPos< gradFieldrows ; rowPos++)
    {
        for(int colPos=0; colPos< gradFieldcols ; colPos++)
        {
            ofFill();
            ofPushMatrix();
            // add half resolution to each dimension to put us in center of each 'cell'
            ofTranslate((colPos*gradFieldresolution) + gradFieldresolution/2,rowPos*gradFieldresolution  + gradFieldresolution/2);
            ofVec2f v1(1,0);
            ofVec2f v2 = gradField[colPos + (rowPos * gradFieldcols)];
            //float angleToRotate = v1.angle(v2);
            //ofRotate(angleToRotate);
            drawArrow(v2*0.1);//(colPos + (rowPos * gradFieldcols))/10);//
            ofSetColor(0,0,255,255);
            //int angFloored = angleToRotate;
            // ofDrawBitmapString(ofToString(angFloored), 0,0); // uncomment to output angle at position for debugging
            ofPopMatrix();
        }
    }
}

void FrameFilter::drawArrow(ofVec2f v1)
{
    // half the length is subtracted from the x point positions to move the rotation axis to the center
    ofSetColor(255,0,0,255);
    ofDrawLine(0, 0, v1.x, v1.y);
    ofDrawCircle(v1.x, v1.y, 5);//(-length/2 + length*0.8, length*0.1, length/2, 0);
    //ofDrawLine(-length/2 + length*0.8, length*-0.1, length/2, 0);
}

ofFloatPixels FrameFilter::filter(ofShortPixels inputframe)
{
    // Create a new output frame: */
    ofFloatPixels newOutputFrame;
    newOutputFrame.allocate(width, height, 1);
    
    if (bufferInitiated)
    {
        // Enter the new frame into the averaging buffer and calculate the output frame's pixel values: */
        const RawDepth* ifPtr=static_cast<const RawDepth*>(inputframe.getData());
        float* abPtr=averagingBuffer+averagingSlotIndex*height*width;
        float* sPtr=statBuffer;
        float* ofPtr=validBuffer; // static_cast<const float*>(outputFrame.getBuffer());
        float* nofPtr=static_cast<float*>(newOutputFrame.getData());
        float z;
        ofVec3f point;
        for(unsigned int y=0;y<height;++y)
        {
            float py=float(y)+0.5f;
            for(unsigned int x=0;x<width;++x,++ifPtr,++abPtr,sPtr+=3,++ofPtr,++nofPtr)
            {
                float px=float(x)+0.5f;
                if(isInsideROI(x, y)) // Check if pixel is inside ROI
                {
                    float oldVal=*abPtr;
                    RawDepth newValRD = *ifPtr;
                    float newVal = (float) newValRD/depthNorm;
                    
                    /* Plug the depth-corrected new value into the minimum and maximum plane equations to determine its validity: */
                    point[0] = px;
                    point[1] = py;
                    point[2] = newVal;
                    bool overMin = classifyPointWithPlane(point, basePlaneNormal, minPlane) == FRONT;
                    bool underMax = classifyPointWithPlane(point, basePlaneNormal, maxPlane) == BACK;
                    
                    if(true)//TODO: add vertical planes limitation
                        //           if(newVal != 0 && newVal != 255) // Pixel depth not clipped => inside valide range
                    {
                        /* Store the new input value: */
                        *abPtr=newVal;
                        
                        /* Update the pixel's statistics: */
                        ++sPtr[0]; // Number of valid samples
                        sPtr[1]+=newVal; // Sum of valid samples
                        sPtr[2]+=newVal*newVal; // Sum of squares of valid samples
                        
                        /* Check if the previous value in the averaging buffer was valid: */
                        if(oldVal!= unvalidValue)
                        {
                            --sPtr[0]; // Number of valid samples
                            sPtr[1]-=oldVal; // Sum of valid samples
                            sPtr[2]-=oldVal*oldVal; // Sum of squares of valid samples
                        }
                    }
                    else if(!retainValids)
                    {
                        /* Store an invalid input value: */
                        *abPtr=unvalidValue;
                        
                        /* Check if the previous value in the averaging buffer was valid: */
                        if(oldVal!=unvalidValue)
                        {
                            --sPtr[0]; // Number of valid samples
                            sPtr[1]-=oldVal; // Sum of valid samples
                            sPtr[2]-=oldVal*oldVal; // Sum of squares of valid samples
                        }
                    }
                    float s0 = sPtr[0];
                    float s1 = sPtr[1];
                    float s2 = sPtr[2];
                    // Check if the pixel is considered "stable": */
                    if(sPtr[0]>=minNumSamples && sPtr[2]*sPtr[0]<=maxVariance*sPtr[0]*sPtr[0]+sPtr[1]*sPtr[1])
                    {
                        /* Check if the new depth-corrected running mean is outside the previous value's envelope: */
                        float newFiltered=float(sPtr[1])/float(sPtr[0]);
                        if(abs(newFiltered-*ofPtr)>=hysteresis)
                        {
                            /* Set the output pixel value to the depth-corrected running mean: */
                            *nofPtr=*ofPtr=newFiltered;
                            firstImageReady = true;
                        }
                        else
                        {
                            /* Leave the pixel at its previous value: */
                            *nofPtr=*ofPtr;
                        }
                    }
                    else if(retainValids)
                    {
                        /* Leave the pixel at its previous value: */
                        *nofPtr=*ofPtr;
                    }
                    else
                    {
                        /* Assign default value to instable pixels: */
                        *nofPtr=instableValue;
                    }
                }
            }
        }
        
        /* Go to the next averaging slot: */
        if(++averagingSlotIndex==numAveragingSlots)
            averagingSlotIndex=0;
        
        /* Apply a spatial filter if requested: */
        if(spatialFilter)
        {
            applySpaceFilter(newOutputFrame);
        }
        
   }
    
    outputframe=newOutputFrame;
    
    // Update gradient field
    updateGradientField();

    return newOutputFrame;
}

void FrameFilter::applySpaceFilter(ofFloatPixels& newOutputFrame)
{
    for(int filterPass=0;filterPass<2;++filterPass)
    {
        /* Low-pass filter the entire output frame in-place: */
        for(unsigned int x=minX;x<maxX;++x)
        {
            /* Get a pointer to the current column: */
            float* colPtr=static_cast<float*>(newOutputFrame.getData())+x;
            
            /* Filter the first pixel in the column: */
            float lastVal=*colPtr;
            *colPtr=(colPtr[0]*2.0f+colPtr[width])/3.0f;
            colPtr+=width;
            
            /* Filter the interior pixels in the column: */
            for(unsigned int y=minY+1;y<maxY-1;++y,colPtr+=width)
            {
                /* Filter the pixel: */
                float nextLastVal=*colPtr;
                *colPtr=(lastVal+colPtr[0]*2.0f+colPtr[width])*0.25f;
                lastVal=nextLastVal;
            }
            
            /* Filter the last pixel in the column: */
            *colPtr=(lastVal+colPtr[0]*2.0f)/3.0f;
        }
        float* rowPtr=static_cast<float*>(newOutputFrame.getData());
        for(unsigned int y=minY;y<maxY;++y)
        {
            /* Filter the first pixel in the row: */
            float lastVal=*rowPtr;
            *rowPtr=(rowPtr[0]*2.0f+rowPtr[1])/3.0f;
            ++rowPtr;
            
            /* Filter the interior pixels in the row: */
            for(unsigned int x=minX+1;x<maxX-1;++x,++rowPtr)
            {
                /* Filter the pixel: */
                float nextLastVal=*rowPtr;
                *rowPtr=(lastVal+rowPtr[0]*2.0f+rowPtr[1])*0.25f;
                lastVal=nextLastVal;
            }
            
            /* Filter the last pixel in the row: */
            *rowPtr=(lastVal+rowPtr[0]*2.0f)/3.0f;
            ++rowPtr;
        }
    }
}

void FrameFilter::updateGradientField()
{
    //Compute gradient field
    int ind = 0;
    float gx;
    float gy;
    int gvx, gvy;
    float lgth = 0;
    float* nofPtr=outputframe.getData();
    for(unsigned int y=0;y<gradFieldrows;++y) {
        for(unsigned int x=0;x<gradFieldcols;++x) {
            gx = 0;
            gvx = 0;
            gy = 0;
            gvy = 0;
            for (unsigned int i=0; i<gradFieldresolution; i++) {
                ind = y*gradFieldresolution*width+i*width+x*gradFieldresolution;
                if (nofPtr[ind]!= 0 && nofPtr[ind+gradFieldresolution-1]!=0){
                    gvx+=1;
                    gx+=nofPtr[ind]-nofPtr[ind+gradFieldresolution-1];
                }
                ind = y*gradFieldresolution*width+i+x*gradFieldresolution;
                if (nofPtr[ind]!= 0 && nofPtr[ind+(gradFieldresolution-1)*width]!=0){
                    gvy+=1;
                    gy+=nofPtr[ind]-nofPtr[ind+(gradFieldresolution-1)*width];
                }
            }
            if (gvx !=0 && gvy !=0)
                gradField[y*gradFieldcols+x]=ofVec2f(gx/gradFieldresolution/gvx*depthrange, gy/gradFieldresolution/gvy*depthrange);
            if (gradField[y*gradFieldcols+x].length() > maxgradfield){
                gradField[y*gradFieldcols+x].scale(maxgradfield);// /= gradField[y*gradFieldcols+x].length()*maxgradfield;
                lgth+=1;
            }
        }
    }
}


//void FrameFilter::setValidDepthInterval(unsigned int newMinDepth,unsigned int newMaxDepth)
//{
//	/* Set the equations for the minimum and maximum plane in depth image space: */
////	minPlane[0]=0.0f;
////	minPlane[1]=0.0f;
////	minPlane[2]=1.0f;
////	minPlane[3]=-float(newMinDepth)+0.5f;
////	maxPlane[0]=0.0f;
////	maxPlane[1]=0.0f;
////	maxPlane[2]=1.0f;
////	maxPlane[3]=-float(newMaxDepth)-0.5f;
//}

void FrameFilter::setValidElevationInterval(const ofVec3f sbasePlaneNormal,double newMinElevation,double newMaxElevation)
{
	/* Calculate the equations of the minimum and maximum elevation planes in camera space: */
    basePlaneNormal = sbasePlaneNormal;
    minPlane=newMinElevation;
    maxPlane=newMaxElevation;
    
    //	PTransform::HVector minPlaneCc(basePlane.getNormal());
    //	minPlaneCc[3]=-(basePlane.getOffset()+newMinElevation*basePlane.getNormal().mag());
    //	PTransform::HVector maxPlaneCc(basePlane.getNormal());
    //	maxPlaneCc[3]=-(basePlane.getOffset()+newMaxElevation*basePlane.getNormal().mag());
    //
    //	/* Transform the plane equations to depth image space and flip and swap the min and max planes because elevation increases opposite to raw depth: */
    //	PTransform::HVector minPlaneDic(depthProjection.getMatrix().transposeMultiply(minPlaneCc));
    //	double minPlaneScale=-1.0/Geometry::mag(minPlaneDic.toVector());
    //	for(int i=0;i<4;++i)
    //		maxPlane[i]=float(minPlaneDic[i]*minPlaneScale);
    //	PTransform::HVector maxPlaneDic(depthProjection.getMatrix().transposeMultiply(maxPlaneCc));
    //	double maxPlaneScale=-1.0/Geometry::mag(maxPlaneDic.toVector());
    //	for(int i=0;i<4;++i)
    //		minPlane[i]=float(maxPlaneDic[i]*maxPlaneScale);
}

void FrameFilter::setStableParameters(unsigned int newMinNumSamples,unsigned int newMaxVariance)
{
    minNumSamples=newMinNumSamples;
    maxVariance=newMaxVariance;
}

void FrameFilter::setHysteresis(float newHysteresis)
{
    hysteresis=newHysteresis;
}

void FrameFilter::setRetainValids(bool newRetainValids)
{
    retainValids=newRetainValids;
}

void FrameFilter::setInstableValue(float newInstableValue)
{
    instableValue=newInstableValue;
}

void FrameFilter::setSpatialFilter(bool newSpatialFilter)
{
    spatialFilter=newSpatialFilter;
}
