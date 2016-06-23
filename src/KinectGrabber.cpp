/*
 * KinectGrabber.cpp
 */

#include "KinectGrabber.h"
#include "ofConstants.h"

//--------------------------------------------------------------
KinectGrabber::KinectGrabber()
:newFrame(true), bufferInitiated(false)
{
}

//--------------------------------------------------------------
KinectGrabber::~KinectGrabber(){
	waitForThread(true);
    
    delete[] averagingBuffer;
    delete[] statBuffer;
    delete[] validBuffer;
    delete[] gradField;
}

//--------------------------------------------------------------
void KinectGrabber::setup(General_state sGS, Calibration_state sCS){
    
	// settings and defaults
	storedframes = 0;
    generalState = sGS;
    calibrationState = sCS;
    
    kinect.init();
    kinect.setRegistration(true); // So we have correspondance between RGB and depth images
    kinect.open();
    kinect.setUseTexture(false);
    width = kinect.getWidth();
    height = kinect.getHeight();
    
    kinectDepthImage.allocate(width, height, 1);
    filteredframe.allocate(width, height, 1);
    kinectColorImage.allocate(width, height);
    kinectColorImage.setUseTexture(false);
}

//--------------------------------------------------------------
void KinectGrabber::setupFramefilter(int sgradFieldresolution, float newMaxOffset, ofRectangle ROI) {
    gradFieldresolution = sgradFieldresolution;
	
	//Framefilter parameters
    numAveragingSlots = 30;
    minNumSamples=(numAveragingSlots+1)/2;
    maxVariance = 4 ;/// depthNorm/depthNorm;
    hysteresis = 0.5f ;/// depthNorm;
    bigChange = 10.0f ;/// depthNorm;
	instableValue=0.0;
    maxgradfield = 1000;
    unvalidValue = 4000;
    spatialFilter = true;
    minInitFrame = 60;
    
    maxOffset =newMaxOffset;
    game = false;
    
    setKinectROI(ROI);
    
    //setting buffers
	initiateBuffers();
}

//--------------------------------------------------------------
void KinectGrabber::initiateBuffers(void){
    
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
    
	/* Initialize the gradient field vector*/
    std::cout<< "Gradient Field resolution" << gradFieldresolution <<std::endl;
    gradFieldcols = width / gradFieldresolution;
    std::cout<< "Width: " << width << " Cols: " << gradFieldcols <<std::endl;
    gradFieldrows = height / gradFieldresolution;
    std::cout<< "Height: " << height << " Rows: " << gradFieldrows <<std::endl;
    
    bufferInitiated = true;
    currentInitFrame = 0;
    firstImageReady = false;
}

//--------------------------------------------------------------
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

//--------------------------------------------------------------
void KinectGrabber::setMode(General_state sgeneralState, Calibration_state scalibrationState){
    generalState = sgeneralState;
    calibrationState = scalibrationState;
    resetBuffers();
}
//--------------------------------------------------------------
void KinectGrabber::setGame(ofPoint smotherRabbit, ofPoint smotherFish, int stype, int splateformSize, bool sgame){
    motherRabbit = smotherRabbit;
    motherFish = smotherFish;
    type = stype;
    plateformSize = splateformSize;
    game = sgame;
}

//--------------------------------------------------------------
ofMatrix4x4 KinectGrabber::getWorldMatrix(){
    ofVec3f a = kinect.getWorldCoordinateAt(0, 0, 1);//*depthNorm; // Trick to access kinect internal parameters without having to modify ofxKinect
    ofVec3f b = kinect.getWorldCoordinateAt(1, 1, 1);//*depthNorm;
    cout << "Computing kinect world matrix" << endl;
    return ofMatrix4x4(b.x-a.x, 0,          0,  a.x,
                       0,       b.y-a.y,    0,  a.y,
                       0,       0,          0,  1/*depthNorm*/,
                       0,       0,          0,  1);
}

//--------------------------------------------------------------
void KinectGrabber::threadedFunction(){
	while(isThreadRunning()) {
        
        //Update state of kinect if needed
        General_state sGS;
        Calibration_state sCS;
        if(generalStateChannel.tryReceive(sGS) || calibrationStateChannel.tryReceive(sCS)) {
            while(generalStateChannel.tryReceive(sGS) || calibrationStateChannel.tryReceive(sCS)) {
            } // clear queue
            setMode(sGS, sCS);
        }

        // If new image in kinect => send to filter thread
        kinect.update();
        
        if(kinect.isFrameNew()){
            kinectDepthImage = kinect.getRawDepthPixels();
            kinectColorImage.setFromPixels(kinect.getPixels());
            filter();
            filteredframe.setImageType(OF_IMAGE_GRAYSCALE);
        }
        if (storedframes == 0)
        {
#if __cplusplus>=201103
            filtered.send(std::move(filteredframe));
            if (generalState == GENERAL_STATE_GAME1)
                gradient.send(std::move(gradField));
            if (generalState == GENERAL_STATE_CALIBRATION)
                colored.send(std::move(kinectColorImage.getPixels()));
#else
            filtered.send(filteredframe);
            if (generalState == GENERAL_STATE_SANDBOX)
                gradient.send(framefilter.getGradField());
            if (generalState == GENERAL_STATE_CALIBRATION)
                colored.send(kinectColorImage.getPixels());
#endif
            lock();
            storedframes += 1;
            unlock();
        }

    }
    kinect.close();
}

//--------------------------------------------------------------
void KinectGrabber::filter()
{
    if (bufferInitiated)
    {
        // Enter the new frame into the averaging buffer and calculate the output frame's pixel values: */
        const RawDepth* ifPtr=static_cast<const RawDepth*>(kinectDepthImage.getData());
        float* abPtr=averagingBuffer+averagingSlotIndex*height*width;
        float* sPtr=statBuffer;
        float* ofPtr=validBuffer; // static_cast<const float*>(outputFrame.getBuffer());
        float* nofPtr=static_cast<float*>(filteredframe.getData());
        float z;
        float animal;
        ofVec3f point;
        for(unsigned int y=0;y<height;++y)
        {
            float py=float(y)+0.5f;
            for(unsigned int x=0;x<width;++x,++ifPtr,++abPtr,sPtr+=3,++ofPtr,++nofPtr)
            {
                float px=float(x)+0.5f;
                if(isInsideROI(x, y)) // Check if pixel is inside ROI
                {
                    RawDepth newValRD = *ifPtr;
                    animal = isInsideAnimalPlateform(x, y);
                    if (animal != 0.0f){
                        newValRD = animal; // We are on an animal plateform
                    }
                    float oldVal=*abPtr;
                    float newVal = (float) newValRD;///depthNorm;
                    
                    /* Plug the depth-corrected new value into the minimum and maximum plane equations to determine its validity: */
                    point[0] = px;
                    point[1] = py;
                    point[2] = newVal;
                    
                    if(newVal>maxOffset)//we are under the max offset plane (to avoid taking into account the hands of operators
                        //           if(newVal != 0 && newVal != 255) // Pixel depth not clipped => inside valide range
                    {
                        /* Store the new input value: */
                        *abPtr=newVal;
                        
                        //Check if there is a big change
                        float oldFiltered = newVal;
                        if (sPtr[0] > 0)
                            oldFiltered =sPtr[1]/sPtr[0];
                        if(abs(oldFiltered-newVal)>=bigChange)
                        {
                            float* aabPtr;
                            // update all averaging slots
                            for (int i = 0; i < numAveragingSlots; i++){
                                aabPtr=averagingBuffer+i*height*width+y*width+x;
                                *aabPtr =newVal;
                            }
                            //Update statistics
                            sPtr[0] = numAveragingSlots;
                            sPtr[1] = newVal*numAveragingSlots;
                            sPtr[2] = newVal*newVal*numAveragingSlots;
                        } else {
                        
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
                    }
//                    if(sPtr[0]>=minNumSamples && sPtr[2]*sPtr[0]<=maxVariance*sPtr[0]*sPtr[0]+sPtr[1]*sPtr[1])
//                    {
//                        float newFiltered=sPtr[1]/sPtr[0];
//                        /* Check if the new depth-corrected running mean is outside the previous value's envelope: */
//                        if(abs(newFiltered-*ofPtr)>=hysteresis)
//                        {
//                            /* Set the output pixel value to the depth-corrected running mean: */
//                            *nofPtr=*ofPtr=newFiltered;
//                        }
//                        else
//                        {
//                            /* Leave the pixel at its previous value: */
//                            *nofPtr=*ofPtr;
//                        }
//                    }
                    // Check if the pixel is considered "stable": */
                    if(sPtr[0]>=minNumSamples && sPtr[2]*sPtr[0]<=maxVariance*sPtr[0]*sPtr[0]+sPtr[1]*sPtr[1])
                    {
                        /* Check if the new depth-corrected running mean is outside the previous value's envelope: */
                        float newFiltered=float(sPtr[1])/float(sPtr[0]);
                        if(abs(newFiltered-*ofPtr)>=hysteresis)
                        {
                            /* Set the output pixel value to the depth-corrected running mean: */
                            *nofPtr=*ofPtr=newFiltered;
                        }
                        else
                        {
                            /* Leave the pixel at its previous value: */
                            *nofPtr=*ofPtr;
                        }
                    }
                    *nofPtr=*ofPtr;
                }
                else
                {
                    *nofPtr=unvalidValue;
                }
            }
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
    
    //    outputframe=newOutputFrame;
    
    // Update gradient field
    updateGradientField();
    
    //    return newOutputFrame;
}

//--------------------------------------------------------------
void KinectGrabber::applySpaceFilter()//ofFloatPixels& newOutputFrame)
{
    for(int filterPass=0;filterPass<2;++filterPass)
    {
        /* Low-pass filter the entire output frame in-place: */
        for(unsigned int x=minX;x<maxX;++x)
        {
            /* Get a pointer to the current column: */
            float* colPtr=static_cast<float*>(filteredframe.getData())+x;
            
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
        float* rowPtr=static_cast<float*>(filteredframe.getData());
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

//--------------------------------------------------------------
void KinectGrabber::updateGradientField()
{
    //Compute gradient field
    int ind = 0;
    float gx;
    float gy;
    int gvx, gvy;
    float lgth = 0;
    float* nofPtr=filteredframe.getData();
    for(unsigned int y=0;y<gradFieldrows;++y) {
        for(unsigned int x=0;x<gradFieldcols;++x) {
            if (isInsideROI(x*gradFieldresolution, y*gradFieldresolution) && isInsideROI((x+1)*gradFieldresolution, (y+1)*gradFieldresolution) ){
                if (y == gradFieldrows/2 && x == gradFieldcols/2){
                    
                }
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

//--------------------------------------------------------------
float KinectGrabber::isInsideAnimalPlateform(int x, int y){ // test is x, y is inside an animal plateform
    float back = 0.0f;
    if (false){//game){
        if (type == 0 || type == 2) {// Test fish
            int dx = x-motherFish.x;
            int dy = y-motherFish.y;
            if (abs(dx)<plateformSize && abs(dy)<plateformSize)
                if ((dx*dx+dy*dy)<plateformSize*plateformSize)
                    back = motherFish.z;
        }
        if (type == 1 || type == 2) {// Test fish
            int dx = x-motherRabbit.x;
            int dy = y-motherRabbit.y;
            if (abs(dx)<plateformSize && abs(dy)<plateformSize)
                if ((dx*dx+dy*dy)<plateformSize*plateformSize)
                    back = motherRabbit.z;
        }
    }
    return back;
}

//--------------------------------------------------------------
void KinectGrabber::setKinectROI(ofRectangle ROI){
    minX = (int) ROI.getMinX();
    maxX = (int) ROI.getMaxX();
    minY = (int) ROI.getMinY();
    maxY = (int) ROI.getMaxY();
    ROIwidth = maxX-minX;
    ROIheight = maxY-minY;
}

//--------------------------------------------------------------
bool KinectGrabber::isInsideROI(int x, int y){
    bool result = true;
    if (x<minX||x>maxX||y<minY||y>maxY)
        result = false;
    return result;
}



