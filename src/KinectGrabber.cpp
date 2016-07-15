/*
 * KinectGrabber.cpp
 */

#include "KinectGrabber.h"
#include "ofConstants.h"

KinectGrabber::KinectGrabber()
:newFrame(true), bufferInitiated(false)
{
}

KinectGrabber::~KinectGrabber(){
    //    stop();
    waitForThread(true);
    //	waitForThread(true);
}

/// Start the thread.
void KinectGrabber::start(){
    startThread(true);
}

/// Signal the thread to stop.  After calling this method,
/// isThreadRunning() will return false and the while loop will stop
/// next time it has the chance to.
void KinectGrabber::stop(){
    stopThread();
}

void KinectGrabber::setup(){
	// settings and defaults
	storedframes = 0;

    kinect.init();
    kinect.setRegistration(true); // To have correspondance between RGB and depth images
    kinect.open();
    kinect.setUseTexture(false);
    width = kinect.getWidth();
    height = kinect.getHeight();
    
    kinectDepthImage.allocate(width, height, 1);
    filteredframe.allocate(width, height, 1);
    kinectColorImage.allocate(width, height);
    kinectColorImage.setUseTexture(false);
}

void KinectGrabber::setupFramefilter(int sgradFieldresolution, float newMaxOffset, ofRectangle ROI, bool sspatialFilter, bool sfollowBigChange, int snumAveragingSlots) {
    gradFieldresolution = sgradFieldresolution;
    ofLogVerbose("kinectGrabber") << "setupFramefilter(): Gradient Field resolution: " << gradFieldresolution;
    gradFieldcols = width / gradFieldresolution;
    ofLogVerbose("kinectGrabber") << "setupFramefilter(): Width: " << width << " Gradient Field Cols: " << gradFieldcols;
    gradFieldrows = height / gradFieldresolution;
    ofLogVerbose("kinectGrabber") << "setupFramefilter(): Height: " << height << " Gradient Field Rows: " << gradFieldrows;
    
    spatialFilter = sspatialFilter;
    followBigChange = sfollowBigChange;
    numAveragingSlots = snumAveragingSlots;
    minNumSamples = (numAveragingSlots+1)/2;
    maxOffset = newMaxOffset;

    //Framefilter default parameters
    maxVariance = 4 ;
    hysteresis = 0.5f ;
    bigChange = 10.0f ;
	instableValue = 0.0;
    maxgradfield = 1000;
    initialValue = 4000;
    outsideROIValue = 3999;
    minInitFrame = 60;
    
    //Setup ROI
    setKinectROI(ROI);
    
    //setting buffers
	initiateBuffers();
}

void KinectGrabber::initiateBuffers(void){
    
    averagingBuffer=new float[numAveragingSlots*height*width];
    float* averagingBufferPtr=averagingBuffer;
    for(int i=0;i<numAveragingSlots;++i)
        for(unsigned int y=0;y<height;++y)
            for(unsigned int x=0;x<width;++x,++averagingBufferPtr)
                *averagingBufferPtr=initialValue; // Mark sample as invalid
    
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
    
    bufferInitiated = true;
    currentInitFrame = 0;
    firstImageReady = false;
}

void KinectGrabber::resetBuffers(void){
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] gradField;
    }
    initiateBuffers();
}

void KinectGrabber::threadedFunction() {
	while(isThreadRunning()) {
        this->actionsLock.lock(); // Update the grabber state if needed
        for(auto & action : this->actions) {
            action(*this);
        }
        this->actions.clear();
        this->actionsLock.unlock();
        
        kinect.update();
        if(kinect.isFrameNew()){
            kinectDepthImage = kinect.getRawDepthPixels();
            filter();
            filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
            updateGradientField();
            kinectColorImage.setFromPixels(kinect.getPixels());
        }
        if (storedframes == 0)
        {
            filtered.send(std::move(filteredframe));
            gradient.send(std::move(gradField));
            colored.send(std::move(kinectColorImage.getPixels()));
            lock();
            storedframes += 1;
            unlock();
        }
        
    }
    kinect.close();
    delete[] averagingBuffer;
    delete[] statBuffer;
    delete[] validBuffer;
    delete[] gradField;
}

void KinectGrabber::performInThread(std::function<void(KinectGrabber&)> action) {
    this->actionsLock.lock();
    this->actions.push_back(action);
    this->actionsLock.unlock();
}

void KinectGrabber::filter()
{
    if (bufferInitiated)
    {
        const RawDepth* inputFramePtr = static_cast<const RawDepth*>(kinectDepthImage.getData());
        float* averagingBufferPtr = averagingBuffer+averagingSlotIndex*height*width;
        float* statBufferPtr = statBuffer;
        float* validBufferPtr = validBuffer;
        float* filteredFramePtr = filteredframe.getData();
        
        inputFramePtr += minY*width;  // We only scan kinect ROI
        averagingBufferPtr += minY*width;
        statBufferPtr += minY*width*3;
        validBufferPtr += minY*width;
        filteredFramePtr += minY*width;
        for(unsigned int y=minY ; y<maxY ; ++y)
        {
            inputFramePtr += minX;
            averagingBufferPtr += minX;
            statBufferPtr += minX*3;
            validBufferPtr += minX;
            filteredFramePtr += minX;
            for(unsigned int x=minX ; x<maxX ; ++x,++inputFramePtr,++averagingBufferPtr,statBufferPtr+=3,++validBufferPtr,++filteredFramePtr)
            {
//                if (x == blockX && y == blockY){
//                    // debug
//                }
                float newVal = static_cast<float>(*inputFramePtr);
                float oldVal = *averagingBufferPtr;
                
                if(newVal > maxOffset)//we are under the ceiling plane
                {
                    *averagingBufferPtr = newVal; // Store the value
                    if (followBigChange && statBufferPtr[0] > 0){ // Follow big changes
                        float oldFiltered = statBufferPtr[1]/statBufferPtr[0]; // Compare newVal with average
                        if(oldFiltered-newVal >= bigChange || newVal-oldFiltered >= bigChange)
                        {
                            float* aaveragingBufferPtr;
                            for (int i = 0; i < numAveragingSlots; i++){ // update all averaging slots
                                aaveragingBufferPtr = averagingBuffer + i*height*width + y*width +x;
                                *aaveragingBufferPtr = newVal;
                            }
                            statBufferPtr[0] = numAveragingSlots; //Update statistics
                            statBufferPtr[1] = newVal*numAveragingSlots;
                            statBufferPtr[2] = newVal*newVal*numAveragingSlots;
                        }
                    }
                    /* Update the pixel's statistics: */
                    ++statBufferPtr[0]; // Number of valid samples
                    statBufferPtr[1] += newVal; // Sum of valid samples
                    statBufferPtr[2] += newVal*newVal; // Sum of squares of valid samples
                    
                    /* Check if the previous value in the averaging buffer was not initiated */
                    if(oldVal != initialValue)
                    {
                        --statBufferPtr[0]; // Number of valid samples
                        statBufferPtr[1] -= oldVal; // Sum of valid samples
                        statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
                    }
                }
                // Check if the pixel is "stable": */
                if(statBufferPtr[0] >= minNumSamples &&
                   statBufferPtr[2]*statBufferPtr[0] <= maxVariance*statBufferPtr[0]*statBufferPtr[0] + statBufferPtr[1]*statBufferPtr[1])
                {
                    /* Check if the new running mean is outside the previous value's envelope: */
                    float newFiltered = statBufferPtr[1]/statBufferPtr[0];
                    if(abs(newFiltered-*validBufferPtr) >= hysteresis)
                    {
                        /* Set the output pixel value to the depth-corrected running mean: */
                        *filteredFramePtr = *validBufferPtr = newFiltered;
                    } else {
                        /* Leave the pixel at its previous value: */
                        *filteredFramePtr = *validBufferPtr;
                    }
                }
                *filteredFramePtr = *validBufferPtr;
            }
            inputFramePtr += width-maxX;
            averagingBufferPtr += width-maxX;
            statBufferPtr += (width-maxX)*3;
            validBufferPtr += width-maxX;
            filteredFramePtr += width-maxX;
        }
        
        /* Go to the next averaging slot: */
        if(++averagingSlotIndex==numAveragingSlots)
            averagingSlotIndex=0;
        
        if (!firstImageReady){
            currentInitFrame++;
            if(currentInitFrame > minInitFrame)
                firstImageReady = true;
        }
        
        /* Apply a spatial filter if requested: */
        if(spatialFilter)
        {
            applySpaceFilter();
        }
    }
}

void KinectGrabber::applySpaceFilter()
{
    for(int filterPass=0;filterPass<2;++filterPass)
    {
        /* Low-pass filter the entire output frame in-place: */
        for(unsigned int x=minX;x<maxX;++x)
        {
            /* Get a pointer to the current column: */
            float* colPtr = filteredframe.getData()+x;
            
            /* Filter the first pixel in the column: */
            float lastVal = *colPtr;
            *colPtr = (colPtr[0]*2.0f+colPtr[width])/3.0f;
            colPtr += width;
            
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
        float* rowPtr = filteredframe.getData();
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

void KinectGrabber::updateGradientField()
{
    int ind = 0;
    float gx;
    float gy;
    int gvx, gvy;
    float lgth = 0;
    float* filteredFramePtr=filteredframe.getData();
    for(unsigned int y=0;y<gradFieldrows;++y) {
        for(unsigned int x=0;x<gradFieldcols;++x) {
            if (isInsideROI(x*gradFieldresolution, y*gradFieldresolution) && isInsideROI((x+1)*gradFieldresolution, (y+1)*gradFieldresolution) ){
                gx = 0;
                gvx = 0;
                gy = 0;
                gvy = 0;
                for (unsigned int i=0; i<gradFieldresolution; i++) {
                    ind = y*gradFieldresolution*width+i*width+x*gradFieldresolution;
                    if (filteredFramePtr[ind]!= 0 && filteredFramePtr[ind+gradFieldresolution-1]!=0){
                        gvx+=1;
                        gx+=filteredFramePtr[ind]-filteredFramePtr[ind+gradFieldresolution-1];
                    }
                    ind = y*gradFieldresolution*width+i+x*gradFieldresolution;
                    if (filteredFramePtr[ind]!= 0 && filteredFramePtr[ind+(gradFieldresolution-1)*width]!=0){
                        gvy+=1;
                        gy+=filteredFramePtr[ind]-filteredFramePtr[ind+(gradFieldresolution-1)*width];
                    }
                }
                if (gvx !=0 && gvy !=0)
                    gradField[y*gradFieldcols+x]=ofVec2f(gx/gradFieldresolution/gvx, gy/gradFieldresolution/gvy);
                if (gradField[y*gradFieldcols+x].length() > maxgradfield){
                    gradField[y*gradFieldcols+x].scale(maxgradfield);// /= gradField[y*gradFieldcols+x].length()*maxgradfield;
                    lgth+=1;
                }
            } else {
                gradField[y*gradFieldcols+x] = ofVec2f(0);
            }
        }
    }
}

bool KinectGrabber::isInsideROI(int x, int y){
    if (x<minX||x>maxX||y<minY||y>maxY)
        return false;
    return true;
}

void KinectGrabber::setKinectROI(ofRectangle ROI){
    minX = (int) ROI.getMinX();
    maxX = (int) ROI.getMaxX();
    minY = (int) ROI.getMinY();
    maxY = (int) ROI.getMaxY();
    ROIwidth = maxX-minX;
    ROIheight = maxY-minY;
    resetBuffers();
}

void KinectGrabber::updateAveragingSlotsNumber(int snumAveragingSlots){
    if (bufferInitiated){
            bufferInitiated = false;
            delete[] averagingBuffer;
            delete[] statBuffer;
            delete[] validBuffer;
            delete[] gradField;
        }
    numAveragingSlots = snumAveragingSlots;
    minNumSamples=(numAveragingSlots+1)/2;
    initiateBuffers();
}

void KinectGrabber::setFollowBigChange(bool newfollowBigChange){
    if (bufferInitiated){
        bufferInitiated = false;
        delete[] averagingBuffer;
        delete[] statBuffer;
        delete[] validBuffer;
        delete[] gradField;
    }
    followBigChange = newfollowBigChange;
    initiateBuffers();
}

ofVec3f KinectGrabber::getStatBuffer(int x, int y){
    float* statBufferPtr = statBuffer+3*(x + y*width);
    return ofVec3f(statBufferPtr[0], statBufferPtr[1], statBufferPtr[2]);
}

float KinectGrabber::getAveragingBuffer(int x, int y, int slotNum){
    float* averagingBufferPtr = averagingBuffer + slotNum*height*width + (x + y*width);
    return *averagingBufferPtr;
}

float KinectGrabber::getValidBuffer(int x, int y){
    float* validBufferPtr = validBuffer + (x + y*width);
    return *validBufferPtr;
}



