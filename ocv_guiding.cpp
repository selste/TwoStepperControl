#include "ocv_guiding.h"
#include "tsc_globaldata.h"
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//---------------------------------------------------
ocv_guiding::ocv_guiding(void) {
    QRgb cval;

    currentImageQImg = new QImage();
    processedImage = new QImage();
    centroidOfGuideStar = new QPoint();
    prevPMap = new QPixmap();
    this->myVec =new QVector<QRgb>(256);
    for(int i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages
    maxX = g_AllData->getCameraChipPixels(0);
    maxY = g_AllData->getCameraChipPixels(1); // get the chip size
}

//---------------------------------------------------
ocv_guiding::~ocv_guiding() {
    delete currentImageQImg;
    delete centroidOfGuideStar;
    delete myVec;
    delete processedImage;
    delete prevPMap;
}

//---------------------------------------------------
void ocv_guiding::convertQImgToMat(void) {
    this->currentImageOCVMat=Mat(this->currentImageQImg->height(),
    this->currentImageQImg->width(), CV_8UC1,
    const_cast<uchar*>(this->currentImageQImg->bits()),
    static_cast<size_t>(this->currentImageQImg->bytesPerLine())).clone();
}

//---------------------------------------------------
void ocv_guiding::convertMatToQImg(void) {
    delete processedImage;
    this->processedImage= new QImage(this->currentImageOCVMat.data,
        this->currentImageOCVMat.cols, this->currentImageOCVMat.rows,
        static_cast<int>(this->currentImageOCVMat.step), QImage::Format_Indexed8);
    this->processedImage->setColorTable(*myVec);
}

//---------------------------------------------------
void ocv_guiding::storeMatToFile(void) {
    convertMatToQImg();
    this->processedImage->save("CurrentOCVImage.jpg",0,-1);
}

//---------------------------------------------------
QPixmap* ocv_guiding::getGuideStarPreview(void) {
    // deliver a small image of the segmented guide star
    return prevPMap;
}

//---------------------------------------------------
void ocv_guiding::doGuideStarImgProcessing(int gsThreshold,bool medianOn,float cntrst, int briteness) {
    int clicx,clicy;
    Point tLeft, bRight;
    float centroidX, centroidY;
    QImage *prevImg;
    cv::Mat mask;
    cv::Moments cvmoms;
    float scaleFact;

    if (g_AllData->getStarSelectionState()==true) {
        delete currentImageQImg;
        this->currentImageQImg = new QImage(*g_AllData->getCameraImage());
        clicx = round(g_AllData->getInitialStarPosition(2));
        clicy = round(g_AllData->getInitialStarPosition(3));
        convertQImgToMat();
        tLeft.x=clicx-90;
        tLeft.y=clicy-90;
        bRight.x=clicx+90;
        bRight.y=clicy+90;
        if (tLeft.x < 0) {
            tLeft.x=0;
        }
        if (tLeft.y < 0) {
            tLeft.y=0;
        }
        if (bRight.x > maxX) {
            bRight.x = maxX;
        }
        if (bRight.y > maxY) {
            bRight.y = maxY;
        }
        Rect R(tLeft,bRight); //Create a rect
        this->currentImageOCVMat = this->currentImageOCVMat(R).clone(); //Crop the region of interest using above rect
        this->currentImageOCVMat.convertTo(this->currentImageOCVMat, -1, cntrst, briteness); // do intensity operations
        cv::threshold(this->currentImageOCVMat,this->currentImageOCVMat, gsThreshold, 255,3); // apply the selected threshold
        if (medianOn== true) {
            cv::medianBlur(this->currentImageOCVMat,this->currentImageOCVMat, 3);
        } // run a 3x3 median filter if desired
        convertMatToQImg();
        prevImg = new QImage(*processedImage);
        prevImg->scaled(180,180,Qt::KeepAspectRatio,Qt::FastTransformation);
        prevPMap->convertFromImage(*prevImg,0);
        delete prevImg;
        cvmoms = moments(this->currentImageOCVMat,true);
        if (cvmoms.m00 > 0.01) {
            centroidX=(cvmoms.m10/(float)cvmoms.m00);
            centroidY=(cvmoms.m01/(float)cvmoms.m00);
            scaleFact=g_AllData->getCameraImageScalingFactor();
            g_AllData->setInitialStarPosition(((tLeft.x+centroidX)*scaleFact),((tLeft.y+centroidY)*scaleFact));
            // this is tricky - correct the position of the manually selected guide star,
            // store this in the global struct and send a signal to the camera view to
            // correct the camera view QGraphicsView ...
            emit guideImagePreviewAvailable();
            emit determinedGuideStarCentroid();
        }
    }
}

//---------------------------------------------------
